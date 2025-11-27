#include "diffdrive_esp/diffdrive_esp.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_esp
{

hardware_interface::CallbackReturn DiffDriveESPHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  device_port_ = info_.hardware_parameters["device_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  timeout_ms_ = std::stoi(info_.hardware_parameters["timeout_ms"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  prev_positions_.resize(info_.joints.size(), 0.0);

  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffDriveESPHardware"),
      "Expected 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("DiffDriveESPHardware"),
        "Joint '%s' has %zu command interfaces found. Expected 1.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("DiffDriveESPHardware"),
        "Joint '%s' has '%s' command interface. Expected '%s'.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("DiffDriveESPHardware"),
        "Joint '%s' has %zu state interfaces. Expected 2.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("DiffDriveESPHardware"),
        "Joint '%s' has '%s' first state interface. Expected '%s'.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("DiffDriveESPHardware"),
        "Joint '%s' has '%s' second state interface. Expected '%s'.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveESPHardware"),
    "Hardware initialized successfully. Device: %s, Baud: %d", 
    device_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveESPHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveESPHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveESPHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveESPHardware"), "Activating hardware...");

  if (!connect())
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveESPHardware"), "Failed to connect to ESP device");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0.0;
      prev_positions_[i] = 0.0;
    }
    if (std::isnan(hw_velocities_[i]))
    {
      hw_velocities_[i] = 0.0;
    }
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveESPHardware"), "Hardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveESPHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveESPHardware"), "Deactivating hardware...");
  
  sendCommand(0.0, 0.0);
  
  disconnect();

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveESPHardware"), "Hardware deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveESPHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  int64_t left_ticks, right_ticks;

  if (readEncoders(left_ticks, right_ticks))
  {
    if (!prev_time_.nanoseconds()) {
      prev_time_ = time;
      prev_left_ticks_ = left_ticks;
      prev_right_ticks_ = right_ticks;
    }

    auto dt = (time - prev_time_).seconds();
    if (dt <= 0.0) dt = 1e-3;

    int64_t delta_left = left_ticks - prev_left_ticks_;
    int64_t delta_right = right_ticks - prev_right_ticks_;

    double delta_theta_left = static_cast<double>(delta_left) / TICKS_PER_REV * 2.0 * M_PI;
    double delta_theta_right = static_cast<double>(delta_right) / TICKS_PER_REV * 2.0 * M_PI;

    hw_positions_[0] += delta_theta_left;
    hw_positions_[1] += delta_theta_right;

    hw_velocities_[0] = delta_theta_left / dt;
    hw_velocities_[1] = delta_theta_right / dt;

    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;
    prev_time_ = time;
  }
  else
  {
    static rclcpp::Clock clock;
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("DiffDriveESPHardware"),
      clock, 2000,
      "No encoder update received - keeping last state");
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveESPHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!sendCommand(hw_commands_[0], hw_commands_[1]))
  {
    static rclcpp::Clock clock;
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("DiffDriveESPHardware"), 
      clock, 1000,
      "Failed to send velocity commands");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool DiffDriveESPHardware::connect()
{
  try
  {
    serial_conn_ = std::make_unique<serial::Serial>(
      device_port_, baud_rate_, serial::Timeout::simpleTimeout(timeout_ms_));
    
    if (!serial_conn_->isOpen())
    {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveESPHardware"), 
                   "Failed to open serial port %s", device_port_.c_str());
      return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    serial_conn_->flushInput();
    serial_conn_->flushOutput();

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveESPHardware"), 
                "Connected to ESP at %s", device_port_.c_str());
    return true;
  }
  catch (const serial::IOException& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveESPHardware"), 
                 "Serial connection error: %s", e.what());
    return false;
  }
}

void DiffDriveESPHardware::disconnect()
{
  if (serial_conn_ && serial_conn_->isOpen())
  {
    serial_conn_->close();
  }
  serial_conn_.reset();
}

bool DiffDriveESPHardware::sendCommand(double left_velocity, double right_velocity)
{
  if (!serial_conn_ || !serial_conn_->isOpen())
  {
    return false;
  }

  try
  {
    double left_rpm = radps_to_rpm(left_velocity);
    double right_rpm = radps_to_rpm(right_velocity);

    // Protocol: "CMD,left_vel,right_vel\n"
    std::string command = "CMD," + std::to_string(left_rpm) + "," + 
                         std::to_string(right_rpm) + "\n";
    return writeLine(command);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveESPHardware"), 
                 "Error sending command: %s", e.what());
    return false;
  }
}

bool DiffDriveESPHardware::readEncoders(int64_t& left_ticks, int64_t& right_ticks)
{
  if (!serial_conn_ || !serial_conn_->isOpen())
    return false;

  try
  {
    if (!writeLine("ENC\n"))
      return false;

    std::string response = readLine();
    if (response.empty())
      return false;

    auto tokens = splitString(response, ',');
    if (tokens.size() != 3 || tokens[0] != "ENC")
    {
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveESPHardware"),
                  "Invalid encoder response: '%s'", response.c_str());
      return false;
    }

    left_ticks = std::stoll(tokens[1]);
    right_ticks = std::stoll(tokens[2]);
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveESPHardware"),
                "Error reading encoders: %s", e.what());
    return false;
  }
}


std::string DiffDriveESPHardware::readLine()
{
  if (!serial_conn_ || !serial_conn_->isOpen())
  {
    return "";
  }

  try
  {
    return serial_conn_->readline(65536, "\n");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveESPHardware"), 
                 "Error reading line: %s", e.what());
    return "";
  }
}

bool DiffDriveESPHardware::writeLine(const std::string& data)
{
  if (!serial_conn_ || !serial_conn_->isOpen())
  {
    return false;
  }

  try
  {
    size_t bytes_written = serial_conn_->write(data);
    return bytes_written == data.size();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DiffDriveESPHardware"), 
                 "Error writing line: %s", e.what());
    return false;
  }
}

double DiffDriveESPHardware::normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double DiffDriveESPHardware::radps_to_rpm(double vel_rad_s)
{
  return (vel_rad_s * 60.0) / (2.0 * M_PI);
}

double DiffDriveESPHardware::rpm_to_radps(double vel_rpm)
{
  return (vel_rpm * 2.0 * M_PI) / 60.0;
}

std::vector<std::string> DiffDriveESPHardware::splitString(const std::string& str, char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;
  
  while (std::getline(ss, token, delimiter))
  {
    // Remove whitespace
    token.erase(token.find_last_not_of(" \n\r\t") + 1);
    token.erase(0, token.find_first_not_of(" \n\r\t"));
    if (!token.empty())
    {
      tokens.push_back(token);
    }
  }
  
  return tokens;
}

}  // namespace diffdrive_esp

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_esp::DiffDriveESPHardware, hardware_interface::SystemInterface)