#include "catamaran_navigator/navigator_sim.hpp"

#include <memory>
#include <string>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace catamaran_navigator
{

NavigatorSim::NavigatorSim()
: Node("navigator_sim")
{
  this->declare_parameter<std::string>("odom_topic", "/catamaran/odometry");
  this->declare_parameter<std::string>("navigator_topic", "/navigator_msg");

  const std::string odom_topic =
    this->get_parameter("odom_topic").as_string();
  const std::string navigator_topic =
    this->get_parameter("navigator_topic").as_string();

  navigator_pub_ = this->create_publisher<sura_msgs::msg::Navigator>(
    navigator_topic, 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    10,
    std::bind(&NavigatorSim::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "NavigatorSim started");
  RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", odom_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing: %s", navigator_topic.c_str());
}

void NavigatorSim::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  sura_msgs::msg::Navigator navigator_msg;

  navigator_msg.position = msg->pose.pose;

  // Velocidad en el frame del cuerpo
  navigator_msg.body_velocity = msg->twist.twist;

  // Quaternion de orientación del robot respecto a NED/world_ned
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  tf2::Matrix3x3 rotation_matrix(q);

  // Velocidad lineal en body
  tf2::Vector3 v_body(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);

  // Transformación de body a NED
  tf2::Vector3 v_ned = rotation_matrix * v_body;

  navigator_msg.ned_velocity.linear.x = v_ned.x();
  navigator_msg.ned_velocity.linear.y = v_ned.y();
  navigator_msg.ned_velocity.linear.z = v_ned.z();

  // De momento copiamos la parte angular tal cual
  navigator_msg.ned_velocity.angular = msg->twist.twist.angular;

  navigator_pub_->publish(navigator_msg);
}

}  // namespace catamaran_navigator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<catamaran_navigator::NavigatorSim>());
  rclcpp::shutdown();
  return 0;
}