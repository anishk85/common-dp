# File: thor_hardware/README.md
# ========================================================================
# Instructions on how to set up and use this package.
#
# Thor Hardware Interface Package

This package provides the necessary software to connect the Thor robot arm to the ROS 2 control framework (`ros2_control`). It is designed to work with a Raspberry Pi as the main computer and an Arduino Mega as a real-time microcontroller for motor and encoder processing.

## Package Contents

-   **`src/thor_arm_hardware_interface.cpp`**: The C++ `ros2_control` hardware interface. This node runs on the Raspberry Pi and communicates with the Arduino over a serial (USB) connection.
-   **`firmware/thor_firmware/thor_firmware.ino`**: The Arduino sketch. This code is responsible for reading the motor encoders, running PID control loops, and driving the motors.
-   **`urdf/thor_arm.ros2_control.xacro`**: A URDF snippet that tells `ros2_control` to load our custom hardware interface plugin.
-   **`config/controllers.yaml`**: Configuration for the ROS 2 controllers (e.g., `joint_trajectory_controller`).
-   **`launch/hardware.launch.py`**: The main launch file to start the physical robot, including `ros2_control` and MoveIt.

## Pre-requisites

1.  **Hardware Assembled:** Your robot arm should be assembled with the goBILDA motors, Raspberry Pi, Arduino Mega, and motor drivers wired correctly.
2.  **Arduino IDE:** You need the Arduino IDE to upload the firmware.
3.  **Serial Library:** The C++ code requires `libserial-dev`. Install it with:
    ```bash
    sudo apt-get update
    sudo apt-get install libserial-dev
    ```

## Setup and Installation

### Step 1: Upload Arduino Firmware

1.  Open the `firmware/thor_firmware/thor_firmware.ino` file in the Arduino IDE.
2.  **CRITICAL:** Before uploading, you **must** update the pin numbers in the `MOTOR_PINS` and `ENCODER_PINS` arrays at the top of the file to match your exact wiring to the Arduino Mega.
3.  Connect the Arduino Mega to your computer via USB.
4.  Select the correct board (Arduino Mega 2560) and port from the `Tools` menu.
5.  Click the "Upload" button.

### Step 2: Update URDF

Your main robot description package (`thor_urdf`) needs to be told to use this hardware interface instead of the Gazebo one.

1.  Open your main `thor_arm.urdf.xacro` file.
2.  Find the line that includes the Gazebo `ros2_control` file, which looks something like this:
    ```xml
    <xacro:include filename="$(find thor_urdf)/urdf/thor_gazebo.xacro" />
    ```
3.  **Comment out** the Gazebo include and **add an include** for the new hardware file:
    ```xml
    <!-- <xacro:include filename="$(find thor_urdf)/urdf/thor_gazebo.xacro" /> -->
    <xacro:include filename="$(find thor_hardware)/urdf/thor_arm.ros2_control.xacro" />
    ```
    *Note: You may want to use a launch argument to switch between simulation and hardware.*

### Step 3: Build the ROS 2 Workspace

Navigate to the root of your workspace and build the new package.

```bash
cd ~/dp_ws/Thor-ROS/ws_thor
colcon build --packages-select thor_hardware
```

## Running the Robot

1.  Power on your robot arm's motor power supply.
2.  Connect the Arduino Mega to the Raspberry Pi via USB.
3.  In a new terminal, source your workspace and run the main hardware launch file:
    ```bash
    source install/setup.bash
    ros2 launch thor_hardware hardware.launch.py
    ```
    This will start the robot driver, `ros2_control`, and MoveIt.
4.  In a second terminal, source your workspace and run your teleop node:
    ```bash
    source install/setup.bash
    ros2 run thor_teleop teleop_pose_control
    ```

Your physical Thor arm should now be controllable via the keyboard.