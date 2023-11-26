#ifndef GUARDIAN__PCA9685_SYSTEM_HPP_
#define GUARDIAN__PCA9685_SYSTEM_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "guardian/pca9685_comm.h"
#include "guardian/visibility_control.h"
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

namespace guardian {

struct SteerJoint
{
  std::string name;
  double position = 0.0;
  int channel = 0;
};


class Pca9685SystemHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  double jt_min_pos_;
  double jt_max_pos_;
  static constexpr int NUM_INTERFACES = 5;
  SteerJoint hw_interfaces_[NUM_INTERFACES];
  
  PCA9685 pca;
  double pca_frequency_;

  void set_servo_pos(int channel, double angle);
};

}  // namespace guardian

#endif  // GUARDIAN__PCA9685_SYSTEM_HPP_
