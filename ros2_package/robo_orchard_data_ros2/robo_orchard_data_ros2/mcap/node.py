# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

import functools
import os
import re
from collections import deque
from datetime import datetime
from queue import Queue

import rclpy
import rosbag2_py
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node, ParameterDescriptor
from rclpy.qos import QoSProfile
from rclpy.serialization import serialize_message
from std_msgs.msg import Header

from robo_orchard_data_ros2.mcap.config import RecordConfig, TopicSpec

__all__ = ["McapRecorder"]


class TopicFilterConflictError(Exception):
    """Exception raised when a topic matches both include and exclude filters."""  # noqa: E501

    pass


class FrameRateMonitor:
    def __init__(self, window_size=30, timeout_threshold: float = 5 * 1e9):
        self.timestamps = deque(maxlen=window_size)
        self.timeout_threshold = timeout_threshold

    def update(self, timestamp_ns: int):
        self.timestamps.append(timestamp_ns)

    def get_fps(self, current_timestamp_ns: int) -> float:
        if (
            len(self.timestamps) < 2
            or current_timestamp_ns - self.timestamps[-1]
            > self.timeout_threshold
        ):
            return 0.0

        time_diff_ns = self.timestamps[-1] - self.timestamps[0]
        if time_diff_ns <= 0:
            return 0.0
        return (len(self.timestamps) - 1) / (time_diff_ns * 1e-9)

    def __len__(self) -> int:
        return len(self.timestamps)


class McapRecorder(Node):
    """A ROS 2 node for recording topics into MCAP format.

    This node subscribes to specified ROS topics, serializes messages,
    and writes them into an MCAP storage format using `rosbag2_py`.
    """

    RECORDING_FILE = "__RECORDING__"

    def __init__(
        self,
    ):
        super().__init__("mcap_recorder")
        self._initialize()
        self._insepct_topics = set()
        self._subscribers = dict()
        self._cnt = 0
        self._hint_freq = 4096
        self._callback_group = ReentrantCallbackGroup()
        self._min_timestamp = None
        self._max_timestamp = None
        self._frame_rate_monitors = dict()
        self._msg_cnt = dict()
        self._static_msgs_buf = Queue()

        if self.config.no_discovery:
            self.scan_topics()
        else:
            self.create_timer(1.0, self.scan_topics)

        self.create_timer(1.0, self._monitor)

    def _monitor(self):
        if self.config.wait_for_topics:
            self.get_logger().warning(
                "Waiting for topics: {}".format(self.config.wait_for_topics)
            )

        """Periodic frame rate validation against configured ranges."""
        for dst_topic, data in self._frame_rate_monitors.items():
            monitor = data["monitor"]

            if self.duration < 5 * 1e9 and len(monitor) < 2:
                return

            spec = data["spec"]
            current_fps = monitor.get_fps(self._max_timestamp)
            min_fps = spec.frame_rate_monitor.min_hz
            max_fps = spec.frame_rate_monitor.max_hz

            # Log warning if outside expected range
            if not (min_fps <= current_fps <= max_fps):
                self.get_logger().warning(
                    f"Topic [{dst_topic}] abnormal frame rate: "
                    f"{current_fps:.2f} Hz "
                    f"(expected range [{min_fps}, {max_fps}])"
                )

    @functools.lru_cache(maxsize=None)  # noqa: B019
    def log_once(self, msg: str, level: str = "info"):
        """Logs a message only once.

        Args:
            msg (str): The message to log.
            level (str): The logging level. Defaults to "info".
        """

        def log_info(msg: str):
            self.get_logger().info(msg)

        def log_error(msg: str):
            self.get_logger().error(msg)

        if level == "info":
            log_info(msg)
        elif level == "error":
            log_error(msg)

    def should_record_topic(self, topic: str) -> bool:
        """Determines whether a topic should be recorded.

        The logic follows these rules:
        1. If no filtering is applied, record all topics by default.
        2. If `include_patterns` exist, only record topics that match.
        3. If `exclude_patterns` exist, filter out matching topics.
        4. If a topic matches both `include` and `exclude`, raise an exception.

        Args:
            topic (str): The name of the topic.

        Returns:
            bool: True if the topic should be recorded, False otherwise.

        Raises:
            TopicFilterConflictError: If the topic matches both include and
                exclude filters.
        """

        if not self._has_topic_filter:
            # by default, record all topics
            return True

        if self.include_topics or self.include_regex:
            include_match = topic in self.include_topics or any(
                regex.match(topic) for regex in self.include_regex
            )
            if not include_match:
                return False
        else:
            include_match = True

        exclude_match = topic in self.exclude_topics or any(
            regex.match(topic) for regex in self.exclude_regex
        )

        if (
            (self.include_topics or self.include_regex)
            and include_match
            and exclude_match
        ):
            exec_msg = (
                f'Topic "{topic}" matches both include and exclude filters, causing a conflict.\n'  # noqa: E501
                f"Include Topics: {self.include_topics}\n"
                f"Exclude Topics: {self.exclude_topics}\n"
                f"Include Regex: {[r.pattern for r in self.include_regex]}\n"
                f"Exclude Regex: {[r.pattern for r in self.exclude_regex]}"
            )
            raise TopicFilterConflictError(exec_msg)

        if exclude_match:
            return False

        return True

    def get_topic_spec(self, topic: str) -> TopicSpec:
        """Retrieves the topic specification for a given topic.

        Args:
            topic (str): The name of the topic.

        Returns:
            TopicSpec: The recording configuration for the topic.
        """
        return self.config.topic_spec.get(
            topic, self.config.default_topic_spec
        )

    def scan_topics(self):
        """Topic scan.

        Scans available topics and subscribes to those that match the filter
        criteria.
        """

        def _get_message_class(msg_type: str):
            """Dynamically imports the message type.

            Args:
                msg_type (str): The ROS 2 message type (e.g.,
                    "std_msgs/msg/String").

            Returns:
                type: The message class, or None if the import fails.
            """
            try:
                module_name, class_name = msg_type.rsplit("/", 1)
                module_name = ".".join(module_name.split("/"))
                module = __import__(module_name, fromlist=[class_name])
                return getattr(module, class_name)
            except Exception as e:
                self.get_logger().error(f"Failed to import {msg_type}: {e}")
                return None

        def _subscribe_topic(topic: str, msg_type_class: type, msg_type: str):
            """Creates a subscription for a given topic.

            Args:
                topic (str): The topic name.
                msg_type_class (type): The message type class associated with
                    the topic.
                msg_type (str): The message type name.
            """
            spec = self.get_topic_spec(topic)
            dst_topic = (
                topic if spec.rename_topic is None else spec.rename_topic
            )
            qos = QoSProfile(
                depth=spec.qos_profile.depth,
                reliability=spec.qos_profile.reliability,
                durability=spec.qos_profile.durability,
                history=spec.qos_profile.history,
            )
            self._subscribers[topic] = self.create_subscription(
                msg_type_class,
                topic,
                functools.partial(
                    self._message_callback,
                    src_topic=topic,
                    dst_topic=dst_topic,
                    spec=spec,
                ),
                qos,
                callback_group=self._callback_group,
            )
            self.writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=dst_topic,
                    type=msg_type,
                    serialization_format="cdr",
                )
            )
            if spec.frame_rate_monitor is not None:
                self._frame_rate_monitors[dst_topic] = {
                    "monitor": FrameRateMonitor(
                        window_size=spec.frame_rate_monitor.window_size
                    ),
                    "spec": spec,
                }
            self._msg_cnt[dst_topic] = 0
            if spec.rename_topic is None:
                self.get_logger().info("Subscribed to topic {}".format(topic))
            else:
                self.get_logger().info(
                    "Subscribed to topic {} and rename to {}".format(
                        topic, spec.rename_topic
                    )
                )

        for topic, msg_type in self.get_topic_names_and_types():
            # cache topic inspect infos
            if topic in self._insepct_topics:
                continue
            self._insepct_topics.add(topic)

            if not self.should_record_topic(topic):
                self.get_logger().info("Ignore topic {}".format(topic))
                continue
            msg_type_class = _get_message_class(msg_type[0])
            if msg_type_class is not None:
                _subscribe_topic(topic, msg_type_class, msg_type[0])
            else:
                self.get_logger().error(
                    "Ignore topic {} because of cannot import the message class".format(  # noqa: E501
                        topic
                    )
                )

    def _write_message(
        self, msg, src_topic: str, dst_topic: str, spec: TopicSpec
    ):
        if (
            spec.stamp_type == "msg_header_stamp"
            and hasattr(msg, "header")
            and isinstance(msg.header, Header)
        ):
            timestamp = int(
                msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            )
        else:
            timestamp = self.get_clock().now().nanoseconds

        if self.config.max_timestamp_difference_ns is not None:
            if self._min_timestamp is None:
                self._min_timestamp = timestamp
            else:
                if (
                    timestamp
                    < self._min_timestamp
                    - self.config.max_timestamp_difference_ns
                ):
                    self.get_logger().error(
                        f"Timestamp of message from {src_topic} exceeds "
                        f"maximum gap ({self.config.max_timestamp_difference_ns} ns). "  # noqa: E501
                        "Dropping message.",
                    )
                    return
                self._min_timestamp = min(timestamp, self._min_timestamp)

            if self._max_timestamp is None:
                self._max_timestamp = timestamp
            else:
                if (
                    self._max_timestamp
                    + self.config.max_timestamp_difference_ns
                    < timestamp
                ):
                    self.get_logger().error(
                        f"Timestamp of message from {src_topic} exceeds "
                        "maximum gap "
                        f"({self.config.max_timestamp_difference_ns} ns). "
                        "Dropping message.",
                    )
                    return
                self._max_timestamp = max(timestamp, self._max_timestamp)

        self.writer.write(dst_topic, serialize_message(msg), timestamp)

        self._cnt += 1

        if dst_topic in self._frame_rate_monitors:
            self._frame_rate_monitors[dst_topic]["monitor"].update(timestamp)

        self._msg_cnt[dst_topic] += 1

        if self._cnt % self._hint_freq == 0:
            self.get_logger().info("Recording {}-th message".format(self._cnt))

    def _flush_caches(self):
        # flush static messages, like /tf_static
        while not self._static_msgs_buf.empty():
            msg, src_topic, dst_topic, spec = self._static_msgs_buf.get()
            self._write_message(msg, src_topic, dst_topic, spec)

    def _message_callback(
        self, msg, src_topic: str, dst_topic: str, spec: TopicSpec
    ):
        """Handles incoming messages and writes them to the MCAP file.

        Args:
            msg: The ROS message.
            src_topic (str): Original topic name.
            dst_topic (str): The writting topic name.
            spec (TopicSpec): The topic spec.
        """
        if self.config.wait_for_topics:
            # store static messages, because these messages will be
            # published once
            if src_topic in self.config.static_topics:
                self._static_msgs_buf.put((msg, src_topic, dst_topic, spec))

            if src_topic in self.config.wait_for_topics:
                self.config.wait_for_topics.remove(src_topic)

            return

        self.log_once("Begining writing...")

        # write flag
        with open(self.recording_flag, "w"):
            pass

        self._flush_caches()
        self._write_message(msg, src_topic, dst_topic, spec)

    def _initialize(self):
        """Recoder Initialization.

        Initializes the recorder by loading configuration and setting up
        MCAP storage.
        """
        self.declare_parameter(
            "config_file",
            "",
            descriptor=ParameterDescriptor(description="Config path"),
        )
        config_file: str = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )
        if config_file == "":
            self.config: RecordConfig = RecordConfig()
        elif os.path.exists(config_file):
            with open(config_file, "r") as handle:
                self.config: RecordConfig = RecordConfig.model_validate_json(  # noqa: E501
                    handle.read()
                )
        else:
            raise FileNotFoundError(
                "config file {} does not exists!".format(config_file)
            )

        self.declare_parameter(
            "uri",
            "",
            descriptor=ParameterDescriptor(
                description="Output bag path, which is a directory"
            ),
        )
        uri: str = self.get_parameter("uri").get_parameter_value().string_value
        if uri == "":
            uri = datetime.now().strftime("rosbag2_%Y_%m_%d-%H_%M_%S")
        self.get_logger().info(f"Recording data to {uri}")

        self.uri = uri
        self.recording_flag = os.path.join(self.uri, self.RECORDING_FILE)
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(
                uri=self.uri,
                storage_id="mcap",
                max_cache_size=self.config.max_cache_size,
            ),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )

        self.include_topics = set()
        self.include_regex = []
        self.exclude_topics = set()
        self.exclude_regex = []

        if self.config.include_patterns:
            for pattern in self.config.include_patterns:
                if "*" in pattern or "." in pattern:
                    self.include_regex.append(re.compile(pattern))
                else:
                    self.include_topics.add(pattern)

        if self.config.exclude_patterns:
            for pattern in self.config.exclude_patterns:
                if "*" in pattern or "." in pattern:
                    self.exclude_regex.append(re.compile(pattern))
                else:
                    self.exclude_topics.add(pattern)

        self._has_topic_filter = (
            self.include_topics
            or self.include_regex
            or self.exclude_topics
            or self.exclude_regex
        )

    def _on_shutdown(self):
        """Performs cleanup tasks when the node is shutting down.

        This includes closing the rosbag writer, removing the recording flag,
        and logging final statistics about the recording session.
        """
        # The destructor of self.writer handles the file closing.
        # Explicitly deleting it ensures it's done in a predictable order.
        del self.writer
        if os.path.exists(self.recording_flag):
            os.remove(self.recording_flag)
        self.get_logger().info(f"Recorded {self._cnt} messages in total.")
        if self.duration == 0:
            msg = (
                "Empty MCAP file detected. Currently wait for topics: "
                f"{self.config.wait_for_topics}\n"
                "Possible causes:\n"
                "1. No matching topics subscribed (current filter: include = "
                f"{self.config.include_patterns} || "
                f"exclude = {self.config.exclude_patterns})\n"
                "2. Publisher nodes terminated before recording started\n"
                "3. QoS incompatibility preventing message reception\n"
                "Diagnostic actions:\n"
                "1. Verify topic availability: ros2 topic list\n"
                "2. Check topic spec compatibility: ros2 topic info\n"
            )
            self.get_logger().warn(msg)
        else:
            duration_sec = self.duration * 1e-9
            self.get_logger().info(
                f"Recording duration: {duration_sec:.2f} seconds."
            )
            for topic, msg_cnt in self._msg_cnt.items():
                self.get_logger().info(
                    "topic {}: count = {}, average frame rate = {:.2f} Hz".format(  # noqa: E501
                        topic,
                        msg_cnt,
                        msg_cnt / duration_sec,
                    )
                )

    @property
    def duration(self) -> int:
        if self._max_timestamp is None or self._min_timestamp is None:
            return 0
        else:
            return self._max_timestamp - self._min_timestamp

    def destroy_node(self):
        self.get_logger().info(
            "Shutting down McapRecorder, performing cleanup..."
        )
        self._on_shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = McapRecorder()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
