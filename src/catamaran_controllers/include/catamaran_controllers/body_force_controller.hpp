#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

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
  static Eigen::Isometry3d urdfPoseToEigen(const urdf::Pose & pose);

  Eigen::Isometry3d jointPoseInBase(
    const urdf::Model & model,
    const std::string & joint_name,
    const std::string & base_link) const;

  bool buildThrusterAllocationMatrix(
    const urdf::Model & model,
    const std::string & base_link,
    const std::vector<std::string> & thruster_joints);

  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd & matrix, double tolerance = 1e-6) const;

  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr body_force_sub_;

  std::string base_link_;
  std::vector<std::string> thruster_joints_;

  Eigen::MatrixXd thruster_allocation_matrix_;

  Eigen::Matrix<double, 6, 1> desired_wrench_ = Eigen::Matrix<double, 6, 1>::Zero();
};

}  // namespace catamaran_controllers