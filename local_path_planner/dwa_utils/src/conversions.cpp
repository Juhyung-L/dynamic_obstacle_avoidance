#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "dwa_utils/conversions.hpp"

namespace dwa_utils
{
nav_2d_msgs::msg::Twist2D twist3Dto2D(const geometry_msgs::msg::Twist& twist)
{
    nav_2d_msgs::msg::Twist2D twist2d;
    twist2d.x = twist.linear.x;
    twist2d.y = twist.linear.y;
    twist2d.theta = twist.angular.z;
    return twist2d;
}

geometry_msgs::msg::Pose2D pose3Dto2D(const geometry_msgs::msg::Pose& pose)
{
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose.position.x;
    pose2d.y = pose.position.y;
    pose2d.theta = tf2::getYaw(pose.orientation);
    return pose2d;
}
}