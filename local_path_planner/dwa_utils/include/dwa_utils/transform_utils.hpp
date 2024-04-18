#ifndef TRANSFORM_UTILS_HPP_
#define TRANSFORM_UTILS_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#include "nav_2d_msgs/msg/path2_d.hpp"

namespace dwa_utils
{
bool transformPath2D(
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::string& frame,
    const nav_2d_msgs::msg::Path2D& in_path,
    nav_2d_msgs::msg::Path2D& out_path,
    const rclcpp::Clock::SharedPtr clock,
    const rclcpp::Duration& transform_tolerance);
}

#endif