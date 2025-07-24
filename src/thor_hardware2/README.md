# Thor Hardware2 - Custom Robotic Arm Hardware Package

This package provides a complete hardware interface for building and controlling a 6-DOF robotic arm using custom components specified in your hardware list.

## Hardware Components

### Control System (₹9,500)
- **Raspberry Pi 5 (4GB)**: Main computer running ROS 2
- **Arduino Mega 2560**: Real-time motor control and sensor reading
- **32GB MicroSD Card**: Storage for OS and software
- **5V 3A USB-C Adapter**: Power supply for Raspberry Pi

### Actuators & Drivers (₹28,000)
- **6x goBILDA 5203 Motors**: High-quality DC motors with integrated encoders
- **3x TB6612FNG Motor Drivers**: H-bridge drivers (2 motors per driver)

### Power & Wiring (₹2,000)
- **5V 10A DC Power Supply**: Motor power source
- **Comprehensive Wiring Kit**: Breadboards, jumper wires, connectors

### Vision System (₹6,000)
- **2x Raspberry Pi Cameras**: Fixed workspace monitoring
- **1x USB Mobile Camera**: Additional perspective/handheld use

### Display System (₹3,000)
- **Raspberry Pi Display**: System monitoring and control interface

**Total Hardware Cost: ₹48,500**
**With Backup Components: ₹60,500**

## Software Architecture

### High-Level Control (ROS 2 on Raspberry Pi)
- Motion planning with MoveIt 2
- Trajectory execution
- Camera data processing
- User interface
- AI policy inference

### Low-Level Control (Arduino Mega)
- Real-time motor control (PID loops)
- Encoder reading and processing
- Safety monitoring
- Serial communication protocol

## Package Structure

```
thor_hardware2/
├── include/           # C++ headers
├── src/              # C++ source files
├── firmware/         # Arduino firmware
├── launch/           # ROS 2 launch files
├── config/           # Configuration files
├── scripts/          # Python nodes
└── README.md
```

## Key Features

### Hardware Interface
- **ros2_control** integration
- Real-time communication with Arduino
- Joint state feedback
- Position command interface
- Safety limits and emergency stop

### Motor Control
- Individual PID control for each joint
- Encoder-based position feedback
- Current monitoring
- Fault detection and recovery

### Camera System
- Multi-camera support (Pi cameras + USB)
- Synchronized image capture
- Camera calibration support
- Real-time video streaming

### Safety Features
- Joint position and velocity limits
- Emergency stop functionality
- Motor fault detection
- Overcurrent protection
- Temperature monitoring

### Monitoring & Diagnostics
- System health monitoring
- Real-time diagnostics
- Motor status reporting
- Power supply monitoring

## Quick Start

### 1. Hardware Assembly
1. Assemble the mechanical structure
2. Mount and connect 6 motors to joints
3. Wire TB6612FNG drivers to Arduino and motors
4. Connect cameras to Raspberry Pi
5. Establish USB communication between Pi and Arduino

### 2. Software Installation
```bash
# Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone <your-repo-url> thor_hardware2

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select thor_hardware2

# Source the workspace
source install/setup.bash
```

### 3. Arduino Firmware Upload
1. Open Arduino IDE
2. Load `firmware/arduino_mega_controller/arduino_mega_controller.ino`
3. Select Arduino Mega 2560 board
4. Upload firmware to Arduino

### 4. Launch the System
```bash
# Launch complete system
ros2 launch thor_hardware2 full_system.launch.py

# Or launch components separately:

# Hardware only
ros2 launch thor_hardware2 hardware.launch.py

# Cameras only  
ros2 launch thor_hardware2 cameras.launch.py
```

## Configuration

### Hardware Configuration (`config/thor_hardware.yaml`)
- Serial port settings
- Joint limits and parameters
- Motor specifications
- Safety parameters

### Controller Configuration (`config/thor_controllers.yaml`)
- Joint trajectory controller settings
- Gripper controller parameters
- Control loop parameters

### Camera Configuration (`config/camera_config.yaml`)
- Camera resolution and framerate
- Calibration parameters
- Device mappings

## Testing

### Motor Testing
```bash
# Test individual motors
ros2 run thor_hardware2 motor_test.py

# Test with different modes
ros2 run thor_hardware2 motor_test.py individual
ros2 run thor_hardware2 motor_test.py sequence
ros2 run thor_hardware2 motor_test.py sine_wave

# Safety tests
ros2 run thor_hardware2 motor_test.py safety

# Calibration routine
ros2 run thor_hardware2 motor_test.py calibration
```

### System Monitoring
```bash
# View system diagnostics
ros2 topic echo /diagnostics

# Monitor system status
ros2 topic echo /system_status

# View joint states
ros2 topic echo /joint_states
```

## Communication Protocol

The package implements a custom serial protocol for Arduino communication:

### Packet Format
```
START_BYTE | COMMAND | LENGTH | DATA | CHECKSUM | END_BYTE
```

### Commands
- `CMD_MOTOR_CONTROL`: Send motor position/velocity commands
- `CMD_READ_SENSORS`: Request motor state data
- `CMD_EMERGENCY_STOP`: Activate emergency stop
- `CMD_SYSTEM_RESET`: Reset Arduino system
- `CMD_HEARTBEAT`: Keep-alive signal
- `CMD_GET_STATUS`: Request system status

## Wiring Diagram

### Motor Connections
```
Arduino Mega → TB6612FNG Drivers → goBILDA Motors

Driver 1 (Pin 9 STBY):
- Motor 0 (Base): PWM=3, DIR1=2, DIR2=4
- Motor 1 (Shoulder): PWM=5, DIR1=7, DIR2=8

Driver 2 (Pin 10 STBY):  
- Motor 2 (Elbow): PWM=6, DIR1=22, DIR2=24
- Motor 3 (Wrist Pitch): PWM=11, DIR1=30, DIR2=32

Driver 3 (Pin 13 STBY):
- Motor 4 (Wrist Roll): PWM=12, DIR1=38, DIR2=40
- Motor 5 (Gripper): PWM=46, DIR1=48, DIR2=50
```

### Encoder Connections
```
Motor 0: A=18, B=19 (Interrupt pins)
Motor 1: A=20, B=21 (Interrupt pins)
Motor 2: A=26, B=28
Motor 3: A=34, B=36
Motor 4: A=42, B=44
Motor 5: A=52, B=53
```

## Troubleshooting

### Common Issues

1. **Arduino not detected**
   - Check USB connection
   - Verify serial port in configuration
   - Ensure firmware is uploaded

2. **Motors not responding**
   - Check power supply connections
   - Verify TB6612FNG wiring
   - Check enable (STBY) pins

3. **Encoder readings incorrect**
   - Verify encoder connections
   - Check for loose wires
   - Ensure proper pull-up resistors

4. **Camera not working**
   - Check camera connections
   - Verify device permissions
   - Test with `v4l2-ctl --list-devices`

### Debugging Commands
```bash
# Check controller manager status
ros2 control list_controllers

# View hardware interface status
ros2 control list_hardware_interfaces

# Monitor serial communication
sudo dmesg | grep tty

# Test camera devices
ls /dev/video*
```

## Future Enhancements

- Force/torque sensor integration
- Advanced gripper control
- Machine learning integration
- Mobile base compatibility
- Wireless operation support

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review system diagnostics
3. Create an issue in the repository
4. Contact the development team

---

**Hardware Package Total Cost: ₹60,500 (including backups)**
**Development Time: ~2-3 weeks**
**Skill Level: Intermediate to Advanced** hardware_interface::CallbackReturn::ERROR;
  }

  // Get serial port parameters
  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  RCLCPP_INFO(
    rclcpp::get_logger("ThorHardwareInterface"),
    "Initializing Thor Hardware Interface with port: %s, baud: %d",
    serial_port_.c_str(), baud_rate_);

  // Initialize joint data vectors
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Initialize joint limits
  initializeJointLimits();

  // Initialize communication
  serial_comm_ = std::make_unique<SerialCommunication>(serial_port_, baud_rate_);

  emergency_stop_ = false;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThorHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ThorHardwareInterface"), "Configuring hardware interface...");

  if (!serial_comm_->connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("ThorHardwareInterface"), "Failed to connect to Arduino");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Send system reset
  if (!serial_comm_->sendSystemReset()) {
    RCLCPP_WARN(rclcpp::get_logger("ThorHardwareInterface"), "Failed to reset Arduino system");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ThorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ThorHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ThorHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ThorHardwareInterface"), "Activating hardware interface...");

  // Initialize commands to current positions
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = hw_positions_[i];
  }

  emergency_stop_ = false;
  last_read_time_ = std::chrono::steady_clock::now();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ThorHardwareInterface"), "Deactivating hardware interface...");

  // Send emergency stop
  serial_comm_->sendEmergencyStop();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ThorHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_comm_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Read motor states from Arduino
  std::vector<MotorState> motor_states(6);
  if (!serial_comm_->readMotorStates(motor_states)) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ThorHardwareInterface"),
      *rclcpp::Clock{RCL_ROS_TIME}.get_clock(),
      1000,
      "Failed to read motor states");
    return hardware_interface::return_type::ERROR;
  }

  // Update joint states
  for (size_t i = 0; i < motor_states.size(); i++) {
    hw_positions_[i] = motor_states[i].position;
    hw_velocities_[i] = motor_states[i].velocity;
    hw_efforts_[i] = motor_states[i].current * MotorSpecs::MAX_EFFORT / 2.0; // Approximate
    
    // Check for motor faults
    if (motor_states[i].fault) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ThorHardwareInterface"),
        "Motor %d fault detected!", static_cast<int>(i));
      emergency_stop_ = true;
    }
  }

  // Send heartbeat
  serial_comm_->sendHeartbeat();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThorHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!serial_comm_->isConnected() || emergency_stop_) {
    return hardware_interface::return_type::ERROR;
  }

  // Validate joint limits
  if (!validateJointLimits()) {
    applyEmergencyStop();
    return hardware_interface::return_type::ERROR;
  }

  // Prepare motor commands
  std::vector<MotorCommand> motor_commands(6);
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    motor_commands[i].motor_id = static_cast<int>(i);
    motor_commands[i].position = hw_commands_[i];
    motor_commands[i].velocity = (hw_commands_[i] - hw_positions_[i]) / period.seconds();
    motor_commands[i].enable = true;
    
    // Clamp velocity
    motor_commands[i].velocity = std::clamp(
      motor_commands[i].velocity,
      -joint_velocity_limits_[i],
      joint_velocity_limits_[i]);
  }

  // Send commands to Arduino
  if (!serial_comm_->sendMotorCommands(motor_commands)) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ThorHardwareInterface"),
      *rclcpp::Clock{RCL_ROS_TIME}.get_clock(),
      1000,
      "Failed to send motor commands");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void ThorHardwareInterface::initializeJointLimits()
{
  // goBILDA 5203 motor limits and typical 6-DOF arm joint limits
  joint_position_limits_min_ = {-3.14, -1.57, -3.14, -3.14, -1.57, -3.14}; // rad
  joint_position_limits_max_ = {3.14, 1.57, 3.14, 3.14, 1.57, 3.14};       // rad
  joint_velocity_limits_ = {3.14, 3.14, 3.14, 6.28, 6.28, 6.28};           // rad/s
  joint_effort_limits_ = {3.5, 3.5, 3.5, 3.5, 3.5, 3.5};                   // Nm
}

bool ThorHardwareInterface::validateJointLimits()
{
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    if (hw_commands_[i] < joint_position_limits_min_[i] || 
        hw_commands_[i] > joint_position_limits_max_[i]) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ThorHardwareInterface"),
        "Joint %zu command %.3f exceeds limits [%.3f, %.3f]",
        i, hw_commands_[i], joint_position_limits_min_[i], joint_position_limits_max_[i]);
      return false;
    }
  }
  return true;
}

void ThorHardwareInterface::applyEmergencyStop()
{
  emergency_stop_ = true;
  serial_comm_->sendEmergencyStop();
  RCLCPP_ERROR(rclcpp::get_logger("ThorHardwareInterface"), "Emergency stop activated!");
}

}  // namespace thor_hardware2

// Plugin export
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  thor_hardware2::ThorHardwareInterface, hardware_interface::SystemInterface)

