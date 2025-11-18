#ifndef DIFFDRIVE_ESP__DIFFDRIVE_ESP_HPP_
#define DIFFDRIVE_ESP__DIFFDRIVE_ESP_HPP_

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <serial/serial.h>

namespace diffdrive_esp
{
class DiffDriveESPHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveESPHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

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
  std::string device_port_;
  int baud_rate_;
  int timeout_ms_;
  
  std::unique_ptr<serial::Serial> serial_conn_;
  
  double wheel_separation_;
  double wheel_radius_;
  static constexpr double TICKS_PER_REV = 11 * 56.0 * 4; 
  
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  
  std::vector<double> prev_positions_;
  rclcpp::Time prev_time_;
  int64_t prev_left_ticks_ = 0;
  int64_t prev_right_ticks_ = 0;
  
  bool connect();
  void disconnect();
  bool sendCommand(double left_velocity, double right_velocity);
  bool readEncoders(int64_t& left_ticks, int64_t& right_ticks);
  std::string readLine();
  bool writeLine(const std::string& data);
  
  double normalizeAngle(double angle);
  double radps_to_rpm(double vel_rad_s);
  double rpm_to_radps(double vel_rpm);
  std::vector<std::string> splitString(const std::string& str, char delimiter);
};

}  // namespace diffdrive_esp

#endif  // DIFFDRIVE_ESP__DIFFDRIVE_ESP_HPP_