#pragma once

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_msgs/msg/navigator.hpp"

namespace catamaran_navigator
{

class NavigatorSim : public rclcpp::Node
{
public:
  NavigatorSim();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sura_msgs::msg::Navigator>::SharedPtr navigator_pub_;
};

}  // namespace catamaran_navigator