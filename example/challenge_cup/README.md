# Running the Full "Challenge Cup" Example

This guide provides the complete procedure for launching all necessary components to perform a full-scale data collection run, replicating the setup used for the official dataset.

You will need to run these commands in separate terminals. Ensure you have sourced your ROS 2 workspace in each terminal before proceeding.

## 1. Launch Sensor and Processing Nodes

First, start the camera and data compression nodes. These will provide the raw and processed data streams for recording.

### a. Start the Camera Node

This command launches the Intel RealSense camera node, which will begin publishing the raw image, depth, and infrared data. For more details on the node's parameters, you can refer to the official [realsense-ros documentation](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#start-the-camera-node).

```bash
# IMPORTANT: Replace '/your/serial_number' with your camera's actual serial number.
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:='agilex' \
    camera_name:='middle_camera' \
    align_depth.enable:=true \
    initial_reset:=true \
    diagnostics_period:=2.0 \
    serial_no:="'/your/serial_number'"
```

### b. Start the Data Compression Nodes

These scripts launch nodes that subscribe to the raw image topics and publish compressed versions. This is crucial for reducing data bandwidth and the final size of the recorded files.

```bash
# In a new terminal, launch the image compression node
bash challenge_cup/launch_image_encoder.sh

# In another new terminal, launch the depth image compression node
bash challenge_cup/launch_depth_encoder.sh
```

### c. Start TF Publish Node
We provide a set of calibration parameters that you can publish as static_tf using the tf node and can view and visualize them through the `/tf_static` topic.
```bash
bash challenge_cup/launch_static_tf_publisher.sh

# then you can get the tf in /tf_static
ros2 topic echo /tf_static
```

## 2. Prepare and Launch the Data Collection App

With the data sources running, you can now prepare and start the main application.

### a. Generate the Data Recorder Configuration

This command creates the specific configuration file that tells the recorder which topics to subscribe to and how to handle them.

```bash
python3 challenge_cup/gen_data_recorder_config.py
```

###  Launch the Data Collection Application

This is the final step to start the user interface.


```bash
bash challenge_cup/launch_app.sh
```

Note: After running this, your terminal will display a URL (e.g., `http://localhost:8501`). Copy this URL and open it in your web browser to access the application.

All recorded data will be saved under the `workspace/challenge_cup/` directory.

## 3. Visualization with Foxglove Layouts

To streamline the visualization process, we provide pre-configured Foxglove layouts for both real-time monitoring and offline analysis.

### For Real-time Visualization

While the system is running, you can monitor the live data streams.

In the Foxglove interface, import the layout file: `challenge_cup/foxglove_layout/websocket.json`

This will automatically set up a dashboard to visualize the key topics from the camera and other sensors in real-time.

### For Offline Data Visualization

After you have recorded an MCAP file, you can use this layout for detailed playback and analysis.

In a standalone Foxglove Studio or in the app's file browser view, open your recorded MCAP file.

Import the layout file: `challenge_cup/foxglove_layout/offline.json`

This provides a comprehensive view tailored for inspecting the recorded data.
