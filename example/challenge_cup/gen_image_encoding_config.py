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

from robo_orchard_data_ros2.codec.image.codec import (
    JpegCodecConfig,
)
from robo_orchard_data_ros2.codec.image.config import ImageEncodingConfig


def main():
    """This script generates a JSON configuration file for the color image encoding node.

    This node subscribes to raw color image topics, compresses them using JPEG, and
    republishes the compressed data on new topics.
    """

    # Define the base namespace for the camera topics.
    # This makes it easy to change if your robot's topic structure is different.
    # For example, if your topics are under "/my_robot/",
    # you would change this value.
    camera_namespace = "/agilex"

    config = ImageEncodingConfig(
        # Specify the compression algorithm to use.
        # Here, we're using JPEG, which is a lossy format that provides
        # excellent compression ratios for color images.
        # This is ideal for reducing bandwidth and storage,
        # trading a small amount of
        # image quality for a significant reduction in size.
        codec=JpegCodecConfig(),
        # Set the number of parallel worker threads for compression.
        # Increasing this on a multi-core CPU can improve performance,
        # but it also increases CPU load.
        # A good starting point is the number of available CPU cores.
        num_workers=8,
        # This is the core of the configuration. It maps input topics (the raw data)
        # to output topics (where the compressed data will be published).
        # The format is: {"input_topic_name": "output_topic_name"}
        topic_mapping={
            f"{camera_namespace}/left_camera/color/image_raw": "/left_camera/color/image_raw/compressed_data",  # noqa: E501
            f"{camera_namespace}/middle_camera/color/image_raw": "/middle_camera/color/image_raw/compressed_data",  # noqa: E501
            f"{camera_namespace}/right_camera/color/image_raw": "/right_camera/color/image_raw/compressed_data",  # noqa: E501
        },
        # Define the maximum number of messages to hold in the input queue.
        # This acts as a buffer to prevent messages from being dropped
        # if there's a temporary spike in data rate
        # or processing load.
        max_queue_size=128,
    )

    # Convert the Python 'config' object into a human-readable JSON string.
    with open("challenge_cup/image_encoding.json", "w") as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
