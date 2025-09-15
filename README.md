# RoboOrchard Data Recorder Toolkit

The RoboOrchard Data Recorder is a comprehensive and robust suite of tools designed for high-fidelity data acquisition in robotics, with a primary focus on the ROS 2 ecosystem. It provides a powerful, configurable backend for recording data into the MCAP format, coupled with a user-friendly web-based application for interactive control and visualization.

This project is built to address the challenges of complex, real-world data collection scenarios, offering fine-grained control over data streams, ensuring data integrity, and simplifying the operator's workflow.

![Architecture](https://github.com/HorizonRobotics/robo_orchard_data_recorder/blob/master/docs/_static/data_collector_arch.png)

## Key Features

1. **Interactive Management with a Web GUI**: A user-friendly application built with Streamlit allows for easy starting, stopping, and monitoring of the recording process. It also features an integrated Foxglove panel for real-time data visualization.

2. **Flexible and Configurable ROS 2 Recording**: The system is built around a versatile Python ROS 2 node that provides a seamless bridge to the MCAP format, offering extensive configuration to handle complex recording scenarios.

3. **Advanced Topic Control**: Go beyond basic recording with powerful configuration options:

- Filter topics using regular expressions (include_patterns, exclude_patterns).

- Define per-topic Quality of Service (QoS) profiles for reliability and durability.

- Dynamically rename topics during recording.

4. **Real-time Data Integrity Monitoring**: Ensure the quality of your collected data with built-in monitoring tools:

- Frame Rate Monitoring: Set minimum and maximum frequency thresholds for any topic to detect sensor dropouts or data floods.

- Timestamp Anomaly Detection: Automatically detect and drop messages with significant timestamp jumps to prevent data corruption.

5. **Robust and Synchronized Recording**:

- Wait for Topics: Configure the recorder to wait for a specified set of topics to become active before starting, ensuring no data is missed at the beginning of a run.

- Static Topic Handling: Properly handles transient local topics (like /tf_static) to ensure they are captured correctly.

6. **Flexible Timestamping**: Choose whether to timestamp messages using the recorder's system clock or the original timestamp from the message header on a per-topic basis.

## Core Components

The repository is organized into three main components:

1. [ROS 2 Recorder Node](https://github.com/HorizonRobotics/robo_orchard_data_recorder/tree/master/ros2_package/robo_orchard_data_ros2)

This is the backbone of the system. It's a ROS 2 package that handles the low-level tasks of subscribing to topics, managing data buffers, and writing to MCAP files. Its behavior is controlled by a detailed JSON configuration file.

2. [Recorder App](https://github.com/HorizonRobotics/robo_orchard_data_recorder/tree/master/python/robo_orchard_recorder_app)

This is the user-facing graphical interface. Built with Streamlit, it provides intuitive controls to manage the recording session. Operators can start/stop recording, monitor system status, and visualize live data streams through the embedded Foxglove interface.

3. [File Server](https://github.com/HorizonRobotics/robo_orchard_data_recorder/tree/master/python/robo_orchard_file_server)

A simple Python-based file server to provide easy access to the recorded MCAP files, which is especially useful when the system is running on a remote robot.

## Getting Started

### Prerequisites

Before you begin, ensure you have the following installed on your system:

- Ubuntu 22.04

- ROS 2 Humble Hawksbill

- Python 3.10+

- `make` build automation tool

### Installation

Clone the repository to your machine

```bash
git clone https://github.com/HorizonRobotics/robo_orchard_data_recorder
cd /path/to/repo
```

This project uses a Makefile to simplify the setup. We recommend using a `Python virtual environment`.

```bash
# prepare development dependency
make dev-env

# Installs the local Python packages in editable mode
make install-editable

# Builds the ROS 2 workspace using colcon
make ros2_clean && make ros2_build
```

### Running an Example

After a successful installation, you can run a [sample application](https://github.com/HorizonRobotics/robo_orchard_data_recorder/tree/master/example/) to see the system in action.

## License

This project is licensed under the Apache 2.0 License. See the LICENSE file for details.