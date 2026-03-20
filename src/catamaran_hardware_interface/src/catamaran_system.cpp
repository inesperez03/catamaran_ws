#include "catamaran_hardware_interface/catamaran_system.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace catamaran_hardware_interface
{

void CatamaranSystem::publish_zero_command()
{
  if (environment_ == "sim" && thruster_stonefish_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {0.0, 0.0};
    thruster_stonefish_pub_->publish(msg);
  }
}

hardware_interface::CallbackReturn CatamaranSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    environment_ = info_.hardware_parameters.at("environment");
    lookup_csv_path_ = info_.hardware_parameters.at("lookup_csv");
  } catch (const std::out_of_range & e) {
    std::cerr << "Missing hardware parameter: " << e.what() << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    std::cerr << "Expected exactly 2 joints, got " << info_.joints.size() << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "effort") {
      std::cerr << "Joint " << joint.name
                << " must have exactly one command interface: effort" << std::endl;
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != "effort") {
      std::cerr << "Joint " << joint.name
                << " must have exactly one state interface: effort" << std::endl;
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  last_left_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_right_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_left_output_ = std::numeric_limits<double>::quiet_NaN();
  last_right_output_ = std::numeric_limits<double>::quiet_NaN();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CatamaranSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!mapper_.loadCsv(lookup_csv_path_)) {
    std::cerr << "Failed to load thruster lookup CSV: " << lookup_csv_path_ << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  is_active_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  if (environment_ == "sim") {
    internal_node_ = std::make_shared<rclcpp::Node>("catamaran_hardware_interface_pub");

    thruster_stonefish_pub_ =
      internal_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/catamaran/controller/thruster_setpoints_sim", 10);

    std::cout << "Stonefish publisher created successfully" << std::endl;
  }

  std::cout << "CatamaranSystem configured successfully" << std::endl;
  std::cout << "Environment: " << environment_ << std::endl;
  std::cout << "Loaded CSV samples: " << mapper_.size() << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CatamaranSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  publish_zero_command();

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  last_left_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_right_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_left_output_ = std::numeric_limits<double>::quiet_NaN();
  last_right_output_ = std::numeric_limits<double>::quiet_NaN();

  std::cout << "CatamaranSystem cleaned up" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CatamaranSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  publish_zero_command();

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  std::cout << "CatamaranSystem shutdown completed" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CatamaranSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = true;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  last_left_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_right_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_left_output_ = std::numeric_limits<double>::quiet_NaN();
  last_right_output_ = std::numeric_limits<double>::quiet_NaN();

  publish_zero_command();

  std::cout << "CatamaranSystem activated" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CatamaranSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  publish_zero_command();

  std::cout << "CatamaranSystem deactivated" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CatamaranSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  publish_zero_command();

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  std::cerr << "CatamaranSystem entered error state" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CatamaranSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "effort", &left_force_state_));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[1].name, "effort", &right_force_state_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CatamaranSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, "effort", &left_force_cmd_));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, "effort", &right_force_cmd_));

  return command_interfaces;
}

hardware_interface::return_type CatamaranSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  left_force_state_ = left_force_cmd_;
  right_force_state_ = right_force_cmd_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CatamaranSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!is_active_) {
    left_force_state_ = 0.0;
    right_force_state_ = 0.0;
    publish_zero_command();
    return hardware_interface::return_type::OK;
  }

  const double left_pwm = mapper_.forceToPwm(left_force_cmd_);
  const double right_pwm = mapper_.forceToPwm(right_force_cmd_);

  const double left_stonefish = mapper_.forceToStonefish(left_force_cmd_);
  const double right_stonefish = mapper_.forceToStonefish(right_force_cmd_);

  if (environment_ == "real") {
    const bool changed =
      std::isnan(last_left_force_cmd_) ||
      std::isnan(last_right_force_cmd_) ||
      std::fabs(left_force_cmd_ - last_left_force_cmd_) > 1e-6 ||
      std::fabs(right_force_cmd_ - last_right_force_cmd_) > 1e-6 ||
      std::fabs(left_pwm - last_left_output_) > 1e-6 ||
      std::fabs(right_pwm - last_right_output_) > 1e-6;

    if (changed) {
      std::cout
        << "[REAL] "
        << "left_force=" << left_force_cmd_ << " N -> left_pwm=" << left_pwm
        << " | right_force=" << right_force_cmd_ << " N -> right_pwm=" << right_pwm
        << std::endl;

      last_left_force_cmd_ = left_force_cmd_;
      last_right_force_cmd_ = right_force_cmd_;
      last_left_output_ = left_pwm;
      last_right_output_ = right_pwm;
    }
  } else if (environment_ == "sim") {
    if (!thruster_stonefish_pub_) {
      std::cerr << "Stonefish publisher not initialized" << std::endl;
      return hardware_interface::return_type::ERROR;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {left_stonefish, right_stonefish};

    thruster_stonefish_pub_->publish(msg);

    const bool changed =
      std::isnan(last_left_force_cmd_) ||
      std::isnan(last_right_force_cmd_) ||
      std::fabs(left_force_cmd_ - last_left_force_cmd_) > 1e-6 ||
      std::fabs(right_force_cmd_ - last_right_force_cmd_) > 1e-6 ||
      std::fabs(left_stonefish - last_left_output_) > 1e-6 ||
      std::fabs(right_stonefish - last_right_output_) > 1e-6;

    if (changed) {
      std::cout
        << "[SIM] "
        << "left_force=" << left_force_cmd_ << " N -> left_stonefish=" << left_stonefish
        << " | right_force=" << right_force_cmd_ << " N -> right_stonefish=" << right_stonefish
        << std::endl;

      last_left_force_cmd_ = left_force_cmd_;
      last_right_force_cmd_ = right_force_cmd_;
      last_left_output_ = left_stonefish;
      last_right_output_ = right_stonefish;
    }
  } else {
    std::cerr << "Unknown environment: " << environment_ << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace catamaran_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  catamaran_hardware_interface::CatamaranSystem,
  hardware_interface::SystemInterface)