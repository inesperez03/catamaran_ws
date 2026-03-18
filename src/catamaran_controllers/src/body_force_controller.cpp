#include "catamaran_controllers/body_force_controller.hpp"

#include <string>

#include "geometry_msgs/msg/wrench.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace catamaran_controllers
{

controller_interface::CallbackReturn BodyForceController::on_init()
{
  auto_declare<std::string>("input_topic", "/body_force/command");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BodyForceController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"left_thruster_joint/effort", "right_thruster_joint/effort"}
  };
}

controller_interface::InterfaceConfiguration
BodyForceController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::NONE,
    {}
  };
}

controller_interface::CallbackReturn BodyForceController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const std::string input_topic =
    get_node()->get_parameter("input_topic").as_string();

  body_force_sub_ = get_node()->create_subscription<geometry_msgs::msg::Wrench>(
    input_topic,
    10,
    [this](const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
      desired_body_force_ = msg->force.x;
    });

  desired_body_force_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyForceController::on_activate(
  const rclcpp_lifecycle::State &)
{
  desired_body_force_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BodyForceController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  desired_body_force_ = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BodyForceController::update(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  const double force_per_thruster = desired_body_force_ / 2.0;

  command_interfaces_[0].set_value(force_per_thruster);
  command_interfaces_[1].set_value(force_per_thruster);

  return controller_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(
  catamaran_controllers::BodyForceController,
  controller_interface::ControllerInterface)