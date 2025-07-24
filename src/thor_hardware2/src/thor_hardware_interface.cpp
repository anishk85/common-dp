#include "thor_hardware2/thor_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace thor_hardware2
{

hardware_interface::CallbackReturn ThorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return data;
}

bool SerialCommunication::deserializeMotorStates(const std::vector<uint8_t>& data, std::vector<MotorState>& states)
{
  if (data.size() != 6 * 17) { // 6 motors * 17 bytes per motor
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunication"), 
                 "Invalid motor state data size: %zu", data.size());
    return false;
  }

  states.resize(6);
  size_t offset = 0;
  
  for (size_t i = 0; i < 6; i++) {
    states[i].motor_id = static_cast<int>(data[offset++]);
    
    // Position (4 bytes)
    float pos;
    memcpy(&pos, &data[offset], 4);
    states[i].position = static_cast<double>(pos);
    offset += 4;
    
    // Velocity (4 bytes)
    float vel;
    memcpy(&vel, &data[offset], 4);
    states[i].velocity = static_cast<double>(vel);
    offset += 4;
    
    // Current (4 bytes)
    float current;
    memcpy(&current, &data[offset], 4);
    states[i].current = static_cast<double>(current);
    offset += 4;
    
    // Encoder count (4 bytes)
    int32_t encoder;
    memcpy(&encoder, &data[offset], 4);
    states[i].encoder_count = encoder;
    offset += 4;
    
    // Fault flag (1 byte)
    states[i].fault = (data[offset++] != 0);
  }
  
  return true;
}

double SerialCommunication::encoderCountsToRadians(int counts)
{
  // goBILDA 5203 has 1440 CPR (counts per revolution)
  return (static_cast<double>(counts) / 1440.0) * 2.0 * M_PI;
}

int SerialCommunication::radiansToEncoderCounts(double radians)
{
  return static_cast<int>((radians / (2.0 * M_PI)) * 1440.0);
}

}  // namespace thor_hardware2
