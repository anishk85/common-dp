# ========================================================================
# File: thor_learning/README.md
# ========================================================================
# Instructions on how to set up and use this package.
#
# Thor AI Learning Package

This package provides the `data_recorder.py` node, which is the primary tool for collecting demonstration data to train an AI policy with LeRobot.

## Features

-   **Joystick Teleoperation:** Control the robot arm's end-effector pose (X, Y, Z, Roll, Pitch, Yaw) intuitively using a PS5 or Xbox controller.
-   **Multi-Camera Support:** Subscribes to two camera topics (`/camera_top/image_raw` and `/camera_base/image_raw`) by default.
-   **Synchronized Data Recording:** Captures images, joint states, end-effector pose, and your joystick commands at a fixed rate (10 Hz).
-   **LeRobot Compatible:** Saves the data directly into an HDF5 file format that LeRobot can use for training without any conversion.

## Pre-requisites

1.  **Joystick:** A PS5 or Xbox controller connected to your computer. The `joy` ROS 2 node must be running.
2.  **Cameras:** Two camera streams must be published on the `/camera_top/image_raw` and `/camera_base/image_raw` topics. You can use the `usb_cam` or `v4l2_camera` packages for this.
3.  **H5Py Library:** The script requires the `h5py` Python library to save dataset files. Install it with:
    ```bash
    pip install h5py
    ```
4.  **Robot is Live:** Your physical robot must be running via the `thor_hardware` launch file, which starts MoveIt and the `ros2_control` drivers.

## Setup and Installation

1.  **Place the Package:** Make sure the `thor_learning` package is inside your workspace's `src` folder.
2.  **Build the Workspace:**
    ```bash
    cd ~/dp_ws/Thor-ROS/ws_thor
    colcon build --packages-select thor_learning
    ```

## How to Record Data

1.  **Launch the Robot:** In one terminal, start your physical robot.
    ```bash
    source install/setup.bash
    ros2 launch thor_hardware hardware.launch.py
    ```

2.  **Launch the Joystick Node:** In a second terminal, start the `joy` node.
    ```bash
    source install/setup.bash
    ros2 run joy joy_node
    ```

3.  **Launch the Camera Nodes:** In one or two more terminals, start your camera drivers. For example:
    ```bash
    # For the top camera
    ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0" -r __ns:=/camera_top
    
    # For the base camera
    ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video2" -r __ns:=/camera_base
    ```
    *(Note: Your `/dev/videoX` numbers may be different.)*

4.  **Launch the Data Recorder:** In a final terminal, run the main script.
    ```bash
    source install/setup.bash
    ros2 run thor_learning data_recorder.py
    ```

5.  **Start Demonstrating!**
    * Use the joystick to control the robot arm.
    * Press the **Start/Options button** on your controller to begin recording an episode. You will see "STARTING RECORDING" in the logs.
    * Perform a complete pick-and-place task.
    * Press the **Start/Options button** again to stop recording. You will see "SAVING Episode" in the logs.
    * Repeat this process to collect many demonstrations (50-100 is a good start).
    * When you are finished, press `Ctrl+C` in the data recorder's terminal. A file named `thor_dataset_... .hdf5` will be in the directory where you ran the command. This is your dataset, ready for training!
