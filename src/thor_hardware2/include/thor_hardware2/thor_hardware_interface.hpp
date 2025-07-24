#ifndef THOR_HARDWARE2__THOR_HARDWARE_INTERFACE_HPP_
#define THOR_HARDWARE2__THOR_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "thor_hardware2/serial_communication.hpp"

namespace thor_hardware2
{
class ThorHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ThorHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware parameters
  std::string serial_port_;
  int baud_rate_;
  
  // Joint data
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  // Communication
  std::unique_ptr<SerialCommunication> serial_comm_;
  
  // Motor specifications for goBILDA 5203
  struct MotorSpecs {
    static constexpr double MAX_VELOCITY = 6.28; // rad/s (1 rev/s)
    static constexpr double MAX_EFFORT = 3.5;    // Nm
    static constexpr int ENCODER_CPR = 1440;     // Counts per revolution
  };
  
  // Safety limits
  std::vector<double> joint_position_limits_min_;
  std::vector<double> joint_position_limits_max_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_effort_limits_;
  
  // Control loop
  bool emergency_stop_;
  std::chrono::steady_clock::time_point last_read_time_;
  
  // Helper methods
  bool validateJointLimits();
  void applyEmergencyStop();
  void initializeJointLimits();
};

}  // namespace thor_hardware2

#endif  // THOR_HARDWARE2__THOR_HARDWARE_INTERFACE_HPP_
