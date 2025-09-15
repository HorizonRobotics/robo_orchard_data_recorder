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

# ruff: noqa: E501

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from robo_orchard_data_ros2.mcap.config import (
    FrameRateMonitor,
    QosProfile,
    RecordConfig,
    TopicSpec,
)


def main():
    """Generates a comprehensive JSON configuration file for the MCAP recorder node.

    This script defines a "production-level" configuration, showcasing advanced
    features for collecting high-quality data from a real-world robotics setup.
    It serves as an excellent template for complex data collection tasks.
    """

    # Define a base namespace for camera topics.
    # This variable makes it easy to
    # adapt the script if your robot uses a different topic prefix (e.g., "/my_robot")
    camera_namespace = "/agilex"

    config = RecordConfig(
        # --- 1. TOPIC SELECTION ---
        # This is a "whitelist" of topics to record. Only topics matching these
        # names will be included in the final MCAP file. This is crucial for
        # controlling the size and content of your dataset.
        include_patterns=[
            # Environment Camera Topics (center, left, right)
            "/middle_camera/color/image_raw/compressed_data",  # image
            "/middle_camera/aligned_depth_to_color/image_raw/compressed_data",  # depth
            f"{camera_namespace}/middle_camera/color/camera_info",  # camera info
            f"{camera_namespace}/middle_camera/aligned_depth_to_color/camera_info",  # camera info
            f"{camera_namespace}/middle_camera/extrinsics/depth_to_color",
            "/left_camera/color/image_raw/compressed_data",  # image
            "/left_camera/aligned_depth_to_color/image_raw/compressed_data",  # depth
            f"{camera_namespace}/left_camera/color/camera_info",  # camera info
            f"{camera_namespace}/left_camera/aligned_depth_to_color/camera_info",  # camera info
            f"{camera_namespace}/left_camera/extrinsics/depth_to_color",
            "/right_camera/color/image_raw/compressed_data",  # image
            "/right_camera/aligned_depth_to_color/image_raw/compressed_data",  # depth
            f"{camera_namespace}/right_camera/color/camera_info",  # camera info
            f"{camera_namespace}/right_camera/aligned_depth_to_color/camera_info",  # camera info
            f"{camera_namespace}/right_camera/extrinsics/depth_to_color",
            # Robot Arm Topics (master and puppet for teleoperation)
            "/master/joint_left",
            "/puppet/joint_left",
            "/puppet/end_pose_left",
            "/master/joint_right",
            "/puppet/joint_right",
            "/puppet/end_pose_right",
            # Core ROS System Topics
            "/tf_static",
            "/tf",
            "/diagnostics",
            "/rosout",
            "/parameter_events",
        ],
        # --- 2. DEFAULT BEHAVIOR ---
        # This section defines the default recording settings for any topic listed in
        # `include_patterns` that does not have a specific override below.
        default_topic_spec=TopicSpec(
            # Default Quality of Service (QoS) settings. QoS is a core ROS 2 feature
            # that controls how messages are handled over the network.
            qos_profile=QosProfile(
                depth=1024,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            # Use the timestamp from the message's header field by default. This is
            # crucial for accurate time synchronization across different sensors.
            stamp_type="msg_header_stamp",
        ),
        # --- 3. SPECIFIC TOPIC OVERRIDES ---
        # This dictionary allows you to define custom settings for specific topics,
        # overriding the defaults set above. This is where the fine-tuning happens.
        topic_spec={
            # --- Center Camera Overrides ---
            "/middle_camera/aligned_depth_to_color/image_raw/compressed_data": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                # Rename the topic in the MCAP file for a cleaner, more organized dataset structure.
                rename_topic="/observation/cameras/middle/depth_image/image_raw",
                # Monitor the frame rate to ensure data quality. Warn if it drops below 25 Hz.
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            "/middle_camera/color/image_raw/compressed_data": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/middle/color_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            f"{camera_namespace}/middle_camera/color/camera_info": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/middle/color_image/camera_info",
            ),
            f"{camera_namespace}/middle_camera/aligned_depth_to_color/camera_info": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/middle/depth_image/camera_info",
            ),
            # This is a static transform. We must override its durability.
            f"{camera_namespace}/middle_camera/extrinsics/depth_to_color": TopicSpec(  # noqa: E501
                rename_topic="/observation/cameras/middle/depth_image/tf",
                # TRANSIENT_LOCAL: This is critical for static transforms. It ensures
                # that even if this message is published only once at startup, any
                # late-joining subscriber (like our recorder) will still receive it.
                qos_profile=QosProfile(
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            ),
            # --- Left Hand Camera Overrides (similar structure to center camera) ---
            "/left_camera/aligned_depth_to_color/image_raw/compressed_data": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/left/depth_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            "/left_camera/color/image_raw/compressed_data": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/left/color_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            f"{camera_namespace}/left_camera/color/camera_info": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/left/color_image/camera_info",
            ),
            f"{camera_namespace}/left_camera/aligned_depth_to_color/camera_info": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/left/depth_image/camera_info",
            ),
            f"{camera_namespace}/left_camera/extrinsics/depth_to_color": TopicSpec(  # noqa: E501
                rename_topic="/observation/cameras/left/depth_image/tf",
                qos_profile=QosProfile(
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            ),
            # --- Right Hand Camera Overrides (similar structure) ---
            "/right_camera/aligned_depth_to_color/image_raw/compressed_data": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/right/depth_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            "/right_camera/color/image_raw/compressed_data": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/right/color_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            f"{camera_namespace}/right_camera/color/camera_info": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/right/color_image/camera_info",
            ),
            f"{camera_namespace}/right_camera/aligned_depth_to_color/camera_info": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/right/depth_image/camera_info",
            ),
            f"{camera_namespace}/right_camera/extrinsics/depth_to_color": TopicSpec(  # noqa: E501
                rename_topic="/observation/cameras/right/depth_image/tf",
                qos_profile=QosProfile(
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            ),
            # --- Robot State Topic Renaming ---
            "/master/joint_left": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/left_master/joint",
            ),
            "/puppet/joint_left": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/left/joint",
            ),
            "/puppet/end_pose_left": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/left/end_pose",
            ),
            "/master/joint_right": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/right_master/joint",
            ),
            "/puppet/joint_right": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/right/joint",
            ),
            "/puppet/end_pose_right": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/right/end_pose",
            ),
            # --- System Topic QoS Overrides ---
            "/tf_static": TopicSpec(
                qos_profile=QosProfile(
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                )
            ),
        },
        # --- 4. DATA INTEGRITY AND SYNCHRONIZATION ---
        # A set of critical topics that the recorder must successfully subscribe to
        # before it starts saving any data. This prevents recordings from starting
        # with missing key data streams.
        wait_for_topics=set(
            (
                # Wait for all camera and robot state messages to be available.
                "/middle_camera/color/image_raw/compressed_data",
                "/middle_camera/aligned_depth_to_color/image_raw/compressed_data",
                "/right_camera/color/image_raw/compressed_data",
                "/right_camera/aligned_depth_to_color/image_raw/compressed_data",
                "/left_camera/color/image_raw/compressed_data",
                "/left_camera/aligned_depth_to_color/image_raw/compressed_data",
                "/puppet/joint_left",
                "/puppet/end_pose_left",
                "/puppet/joint_right",
                "/puppet/end_pose_right",
            )
        ),
        # A list of topics that are known to be static (i.e., published once with
        # TRANSIENT_LOCAL durability). This helps the recorder handle them correctly.
        static_topics=[
            f"{camera_namespace}/middle_camera/extrinsics/depth_to_color",
            f"{camera_namespace}/left_camera/extrinsics/depth_to_color",
            f"{camera_namespace}/right_camera/extrinsics/depth_to_color",
            "/tf_static",
        ],
        # A data quality check. If the timestamp difference between consecutive messages
        # on any topic is greater than this value (0.5 seconds), the message is
        # considered anomalous and dropped. This prevents issues from system clock jumps.
        max_timestamp_difference_ns=0.5 * 1e9,  # 1s
    )

    # Write the configuration object to a JSON file.
    with open("challenge_cup/data_recorder.json", "w") as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
