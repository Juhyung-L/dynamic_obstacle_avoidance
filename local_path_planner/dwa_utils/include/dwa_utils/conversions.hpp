#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "nav_2d_msgs/msg/twist2_d.hpp"

namespace dwa_utils
{
    nav_2d_msgs::msg::Twist2D twist3Dto2D(const geometry_msgs::msg::Twist& twist);
    geometry_msgs::msg::Pose2D pose3Dto2D(const geometry_msgs::msg::Pose& pose);
}

#endif