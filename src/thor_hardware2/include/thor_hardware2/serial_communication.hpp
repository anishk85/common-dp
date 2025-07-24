#ifndef THOR_HARDWARE2__SERIAL_COMMUNICATION_HPP_
#define THOR_HARDWARE2__SERIAL_COMMUNICATION_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <chrono>

#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"

namespace thor_hardware2
{

struct MotorCommand {
  int motor_id;
  double velocity;  // rad/s
  double position;  // rad
  bool enable;
};

struct MotorState {
  int motor_id;
  double position;    // rad
  double velocity;    // rad/s
  double current;     // A
  bool fault;
  int encoder_count;
};

class SerialCommunication
{
public:
  SerialCommunication(const std::string& port, int baud_rate);
  ~SerialCommunication();

  bool connect();
  void disconnect();
  bool isConnected() const;

  // Motor control commands
  bool sendMotorCommands(const std::vector<MotorCommand>& commands);
  bool readMotorStates(std::vector<MotorState>& states);
  
  // System commands
  bool sendEmergencyStop();
  bool sendSystemReset();
  bool sendHeartbeat();
  
  // Diagnostics
  bool getSystemStatus(std::string& status);
  double getVoltage();
  double getCurrent();

private:
  std::unique_ptr<serial::Serial> serial_;
  std::string port_;
  int baud_rate_;
  mutable std::mutex serial_mutex_;
  
  // Communication protocol
  static constexpr char START_BYTE = 0xAA;
  static constexpr char END_BYTE = 0x55;
  static constexpr int MAX_RETRIES = 3;
  static constexpr int TIMEOUT_MS = 100;
  
  // Command types
  enum CommandType {
    CMD_MOTOR_CONTROL = 0x01,
    CMD_READ_SENSORS = 0x02,
    CMD_EMERGENCY_STOP = 0x03,
    CMD_SYSTEM_RESET = 0x04,
    CMD_HEARTBEAT = 0x05,
    CMD_GET_STATUS = 0x06
  };
  
  // Helper methods
  bool sendCommand(uint8_t cmd_type, const std::vector<uint8_t>& data);
  bool receiveResponse(std::vector<uint8_t>& response);
  uint8_t calculateChecksum(const std::vector<uint8_t>& data);
  bool validateResponse(const std::vector<uint8_t>& response);
  
  // Data conversion
  std::vector<uint8_t> serializeMotorCommands(const std::vector<MotorCommand>& commands);
  bool deserializeMotorStates(const std::vector<uint8_t>& data, std::vector<MotorState>& states);
  
  // Encoder conversion (goBILDA 5203 specific)
  double encoderCountsToRadians(int counts);
  int radiansToEncoderCounts(double radians);
};

}  // namespace thor_hardware2

#endif  // THOR_HARDWARE2__SERIAL_COMMUNICATION_HPP_
