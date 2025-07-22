/*
========================================================================
File: thor_hardware/src/thor_arm_hardware_interface.cpp
========================================================================
The core C++ driver that communicates with the Arduino over a serial port.
*/
#include "thor_hardware/thor_arm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <iomanip>

namespace thor_hardware
{

hardware_interface::CallbackReturn ThorArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage for commands and states
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Get parameters from the URDF
  std::string serial_port_name = info_.hardware_parameters["serial_port"];
  int baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  double encoder_cpr = std::stod(info_.hardware_parameters["encoder_cpr"]);
  
  // The goBILDA motor has a 4-stage planetary gearbox. The encoder is on the motor shaft.
  // We need to account for the gear ratio to get the output shaft angle.
  // The specific motor has a 99.5:1 ratio, but let's assume a generic 100:1 for this example.
  // This should be tuned to your exact motor.
  double gear_ratio = 100.0; 
  ticks_to_rads_ = (2 * M_PI) / (encoder_cpr * gear_ratio);

  // Open the serial port
  try {
    serial_port_.Open(serial_port_name);
    serial_port_.SetBaudRate(static_cast<LibSerial::BaudRate>(baud_rate));
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("ThorArmHardwareInterface"), "Failed to open serial port %s: %s", serial_port_name.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ThorArmHardwareInterface"), "Successfully opened serial port %s at %d baud.", serial_port_name.c_str(), baud_rate);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ThorArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ThorArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type ThorArmHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read from the serial port. The Arduino should be sending data in the format:
  // "p <j1_ticks> <j2_ticks> <j3_ticks> <j4_ticks> <j5_ticks> <j6_ticks>\n"
  std::string response;
  try {
    serial_port_.ReadLine(response, '\n', 100); // 100ms timeout
    
    std::stringstream ss(response);
    char prefix;
    ss >> prefix;

    if (prefix == 'p') {
      for (int i = 0; i < 6; ++i) {
        long ticks;
        ss >> ticks;
        hw_positions_[i] = static_cast<double>(ticks) * ticks_to_rads_;
      }
    }
  } catch (const LibSerial::ReadTimeout&) {
    RCLCPP_WARN(rclcpp::get_logger("ThorArmHardwareInterface"), "Serial read timeout.");
    return hardware_interface::return_type::OK; // Don't return error on timeout
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ThorArmHardwareInterface"), "Error reading from serial port: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThorArmHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write the commands to the serial port. Format:
  // "s <j1_rad> <j2_rad> <j3_rad> <j4_rad> <j5_rad> <j6_rad>\n"
  std::stringstream command_stream;
  command_stream << "s ";
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    // If a command hasn't been set yet, use the current position
    double command = std::isnan(hw_commands_[i]) ? hw_positions_[i] : hw_commands_[i];
    command_stream << std::fixed << std::setprecision(4) << command << " ";
  }
  command_stream << "\n";

  try {
    serial_port_.Write(command_stream.str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ThorArmHardwareInterface"), "Error writing to serial port: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace thor_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(thor_hardware::ThorArmHardwareInterface, hardware_interface::SystemInterface)
