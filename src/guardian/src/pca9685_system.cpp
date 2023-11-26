#include "guardian/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace guardian
{
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get Params
  // pca_frequency_ = std::stod(info_.hardware_parameters["pca_frequency"]);
  // jt_min_pos_ = std::stod(info_.hardware_parameters["jt_lower_limit"]);
  // jt_max_pos_ = std::stod(info_.hardware_parameters["jt_upper_limit"]);
  pca_frequency_ = 50.0;
  jt_min_pos_ = 0.0;
  jt_max_pos_ = 3.14159;

  // Check URDF ros2_control xacro params
  int i = 0;
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check Command Interfaces
    // Pca9685System has one command interface on each joint (position)
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s'.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check State Interfaces
    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                   "Joint '%s' has %zu state interface. 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
                   "Joint '%s' have %s state interface. '%s'.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Add joint to the internal structure
    hw_interfaces_[i].name = joint.name;
    hw_interfaces_[i].channel = std::stoi(info_.hardware_parameters[joint.name + "__channel"]);
    // hw_interfaces_[i].position = std::stod(info_.hardware_parameters[joint.name + "__init_position"]);
    i++;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}
  
hardware_interface::CallbackReturn Pca9685SystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
              "Configuring...");

  pca.set_pwm_freq(pca_frequency_);

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
              "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < NUM_INTERFACES; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      hw_interfaces_[i].name, hardware_interface::HW_IF_POSITION, &hw_interfaces_[i].position));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < NUM_INTERFACES; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      hw_interfaces_[i].name, hardware_interface::HW_IF_POSITION, &hw_interfaces_[i].position));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_interfaces_[0].position = 0.0;
  hw_interfaces_[1].position = 3.1;
  hw_interfaces_[2].position = 3.1;
  hw_interfaces_[3].position = 0.0;
  hw_interfaces_[4].position = 0.0;
  
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}

void Pca9685SystemHardware::set_servo_pos(int channel, double angle){
  // Angle is in radians
  double clamped_angle = std::clamp(angle, jt_min_pos_, jt_max_pos_ - 0.001 );
  // Convert the angle to a corresponding pulse width between 0.5 ms and 2.5 ms
  double min_pulse_width = 0.5;
  double max_pulse_width = 2.5;
  double pulse_ms = min_pulse_width + (clamped_angle / jt_max_pos_) * (max_pulse_width - min_pulse_width);  
  
  pca.set_pwm_ms(channel, pulse_ms);
}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  for (std::size_t i = 0; i < NUM_INTERFACES; i++)
  {
    set_servo_pos(hw_interfaces_[i].channel, hw_interfaces_[i].position);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace guardian

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  guardian::Pca9685SystemHardware, hardware_interface::SystemInterface)
