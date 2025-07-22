/*
========================================================================
File: thor_hardware/include/thor_hardware/thor_arm_hardware_interface.hpp
========================================================================
Header file for our hardware interface.
*/
#ifndef THOR_ARM_HARDWARE_INTERFACE_HPP
#define THOR_ARM_HARDWARE_INTERFACE_HPP

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "libserial/SerialPort.h"

namespace thor_hardware
{
class ThorArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ThorArmHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Store commands and states
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;

  // Serial port communication
  LibSerial::SerialPort serial_port_;
  
  // Conversion factor from encoder ticks to radians
  double ticks_to_rads_;
};
}  // namespace thor_hardware

#endif // THOR_ARM_HARDWARE_INTERFACE_HPP