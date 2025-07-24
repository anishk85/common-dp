#include "thor_hardware2/serial_communication.hpp"
#include <algorithm>
#include <thread>

namespace thor_hardware2
{

SerialCommunication::SerialCommunication(const std::string& port, int baud_rate)
  : port_(port), baud_rate_(baud_rate)
{
  serial_ = std::make_unique<serial::Serial>();
}

SerialCommunication::~SerialCommunication()
{
  disconnect();
}

bool SerialCommunication::connect()
{
  try {
    serial_->setPort(port_);
    serial_->setBaudrate(baud_rate_);
    serial_->setTimeout(serial::Timeout::simpleTimeout(TIMEOUT_MS));
    serial_->open();
    
    if (serial_->isOpen()) {
      RCLCPP_INFO(rclcpp::get_logger("SerialCommunication"), 
                  "Successfully connected to %s at %d baud", port_.c_str(), baud_rate_);
      
      // Clear any existing data
      serial_->flush();
      return true;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunication"), 
                 "Failed to connect: %s", e.what());
  }
  return false;
}

void SerialCommunication::disconnect()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_ && serial_->isOpen()) {
    serial_->close();
  }
}

bool SerialCommunication::isConnected() const
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_ && serial_->isOpen();
}

bool SerialCommunication::sendMotorCommands(const std::vector<MotorCommand>& commands)
{
  if (commands.size() != 6) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunication"), 
                 "Invalid number of motor commands: %zu", commands.size());
    return false;
  }

  std::vector<uint8_t> data = serializeMotorCommands(commands);
  return sendCommand(CMD_MOTOR_CONTROL, data);
}

bool SerialCommunication::readMotorStates(std::vector<MotorState>& states)
{
  states.resize(6);
  
  // Send read request
  std::vector<uint8_t> empty_data;
  if (!sendCommand(CMD_READ_SENSORS, empty_data)) {
    return false;
  }

  // Receive response
  std::vector<uint8_t> response;
  if (!receiveResponse(response)) {
    return false;
  }

  return deserializeMotorStates(response, states);
}

bool SerialCommunication::sendEmergencyStop()
{
  std::vector<uint8_t> empty_data;
  return sendCommand(CMD_EMERGENCY_STOP, empty_data);
}

bool SerialCommunication::sendSystemReset()
{
  std::vector<uint8_t> empty_data;
  return sendCommand(CMD_SYSTEM_RESET, empty_data);
}

bool SerialCommunication::sendHeartbeat()
{
  std::vector<uint8_t> empty_data;
  return sendCommand(CMD_HEARTBEAT, empty_data);
}

bool SerialCommunication::sendCommand(uint8_t cmd_type, const std::vector<uint8_t>& data)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  
  if (!serial_->isOpen()) {
    return false;
  }

  try {
    // Build packet: START_BYTE | CMD_TYPE | DATA_LENGTH | DATA | CHECKSUM | END_BYTE
    std::vector<uint8_t> packet;
    packet.push_back(START_BYTE);
    packet.push_back(cmd_type);
    packet.push_back(static_cast<uint8_t>(data.size()));
    
    for (uint8_t byte : data) {
      packet.push_back(byte);
    }
    
    uint8_t checksum = calculateChecksum(data);
    packet.push_back(checksum);
    packet.push_back(END_BYTE);

    size_t bytes_written = serial_->write(packet);
    serial_->flush();
    
    return bytes_written == packet.size();
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunication"), 
                 "Error sending command: %s", e.what());
    return false;
  }
}

bool SerialCommunication::receiveResponse(std::vector<uint8_t>& response)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  
  if (!serial_->isOpen()) {
    return false;
  }

  try {
    // Wait for start byte
    while (serial_->available() > 0) {
      uint8_t byte;
      serial_->read(&byte, 1);
      if (byte == START_BYTE) {
        break;
      }
    }

    // Read response length
    uint8_t response_length;
    if (serial_->read(&response_length, 1) != 1) {
      return false;
    }

    // Read response data
    response.resize(response_length);
    if (serial_->read(response.data(), response_length) != response_length) {
      return false;
    }

    // Read checksum and end byte
    uint8_t received_checksum, end_byte;
    if (serial_->read(&received_checksum, 1) != 1 || 
        serial_->read(&end_byte, 1) != 1) {
      return false;
    }

    // Validate packet
    if (end_byte != END_BYTE) {
      RCLCPP_WARN(rclcpp::get_logger("SerialCommunication"), "Invalid end byte");
      return false;
    }

    uint8_t calculated_checksum = calculateChecksum(response);
    if (received_checksum != calculated_checksum) {
      RCLCPP_WARN(rclcpp::get_logger("SerialCommunication"), "Checksum mismatch");
      return false;
    }

    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialCommunication"), 
                 "Error receiving response: %s", e.what());
    return false;
  }
}

uint8_t SerialCommunication::calculateChecksum(const std::vector<uint8_t>& data)
{
  uint8_t checksum = 0;
  for (uint8_t byte : data) {
    checksum ^= byte;
  }
  return checksum;
}

std::vector<uint8_t> SerialCommunication::serializeMotorCommands(const std::vector<MotorCommand>& commands)
{
  std::vector<uint8_t> data;
  
  for (const auto& cmd : commands) {
    // Motor ID (1 byte)
    data.push_back(static_cast<uint8_t>(cmd.motor_id));
    
    // Position (4 bytes, float)
    float pos = static_cast<float>(cmd.position);
    uint8_t* pos_bytes = reinterpret_cast<uint8_t*>(&pos);
    for (int i = 0; i < 4; i++) {
      data.push_back(pos_bytes[i]);
    }
    
    // Velocity (4 bytes, float)
    float vel = static_cast<float>(cmd.velocity);
    uint8_t* vel_bytes = reinterpret_cast<uint8_t*>(&vel);
    for (int i = 0; i < 4; i++) {
      data.push_back(vel_bytes[i]);
    }
    
    // Enable flag (1 byte)
    data.push_back(cmd.enable ? 1 : 0);
  }
  
  return data;
}