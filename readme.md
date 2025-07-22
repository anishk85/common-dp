# Thor-ROS

## Overview

This repository contains ROS 2 packages for controlling and teleoperating the Thor robotic arm, including MoveIt integration, teleoperation, perception, and more.  
The workspace is organized as a typical ROS 2 workspace under `ws_thor/src/`.

## Key Packages

- **thor_urdf**: URDF/Xacro robot description files for Thor.
- **thor_moveit**: MoveIt configuration for motion planning.
- **thor_controller**: Low-level controllers and hardware interfaces.
- **thor_perception**: Perception nodes for object detection and pick/place.
- **thor_teleop**: Teleoperation nodes for controlling the arm via keyboard or joystick.
- **thor_manipulation**: High-level manipulation logic and demos.

---

## thor_teleop

### Overview

The `thor_teleop` package provides teleoperation interfaces for the Thor arm, including:
- **C++ Node**: `teleop_pose_control` for pose-based control.
- **Python Node**: `teleop_joint_control.py` for joint-level teleop (if present).
- **Python Node**: `teleop_pose_control.py` for Cartesian pose teleop via keyboard.

### Building

From your workspace root:

```bash
cd /home/anish/dp_ws/Thor-ROS/ws_thor
colcon build --packages-select thor_teleop
source install/setup.bash
```

### Usage

#### 1. C++ Pose Teleop Node

```bash
ros2 run thor_teleop teleop_pose_control
```

#### 2. Python Cartesian Pose Teleop Node

```bash
ros2 run thor_teleop teleop_pose_control.py
```

#### 3. Python Joint Teleop Node

If you have `scripts/teleop_joint_control.py`:

```bash
ros2 run thor_teleop teleop_joint_control.py
```

> **Note:**  
> Make sure your Python scripts are executable (`chmod +x scripts/teleop_*.py`) and installed via `CMakeLists.txt` using the `PROGRAMS` keyword.

---

## Example: Keyboard Cartesian Teleop

The Python node `teleop_pose_control.py` allows you to jog the end-effector in XYZ and rotate in roll/pitch/yaw using the keyboard.

**Controls:**

- **Position (XYZ):**
  - Forward/Backward: `w` / `s`
  - Left/Right: `a` / `d`
  - Up/Down: `r` / `f`
- **Orientation:**
  - Roll: `q` / `e`
  - Pitch: `t` / `g`
  - Yaw: `y` / `h`
- **Electromagnet:**  
  - Toggle: `spacebar`
- **Quit:**  
  - `Ctrl+C`

---

## URDF and MoveIt

- The robot description is in [`thor_urdf/urdf/thor.urdf.xacro`](src/thor_urdf/urdf/thor.urdf.xacro).
- MoveIt configuration is in [`thor_moveit`](src/thor_moveit/).

---

## Troubleshooting

- If you get `No executable found` for a Python node, ensure:
  - The script is executable (`chmod +x`).
  - It is installed in `CMakeLists.txt` with `install(PROGRAMS ...)`.
  - You have rebuilt and sourced your workspace.

- If you get CMake errors about dependencies, ensure all `find_package(...)` and `ament_target_dependencies(...)` are correct and match your code.

---

## License

This project is for research and educational use.  
See individual package folders for license details.

---

## Acknowledgements

- ROS 2, MoveIt, and the open-source robotics community.

---

**Maintainer:**  
Anish  