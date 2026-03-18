#pragma once

#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"

namespace catamaran_controllers
{

class BodyForceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr body_force_sub_;
  double desired_body_force_{0.0};
};

}  // namespace catamaran_controllers