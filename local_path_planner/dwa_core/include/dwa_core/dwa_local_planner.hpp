#ifndef DWA_LOCAL_PLANNER_HPP_
#define DWA_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "dwa_core/xytheta_velocity_iterator.hpp"
#include "dwa_core/kinematics_parameters.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/path2_d_list.hpp"

namespace dwa_core
{
class DWALocalPlanner : public nav2_util::LifecycleNode
{
public:
    explicit DWALocalPlanner(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());
    ~DWALocalPlanner();

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
    KinematicsParameters::SharedPtr kp_;
    XYThetaVelocityIterator vel_it_;

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
    nav2_costmap_2d::Costmap2D* costmap_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry odom_;
    std::string odom_topic_;

    rclcpp_lifecycle::LifecyclePublisher<nav_2d_msgs::msg::Path2DList>::SharedPtr trajs_pub_;
    nav_2d_msgs::msg::Path2DList trajs_;
    nav_2d_msgs::msg::Path2D traj_;

    int num_iterations_;
    double sim_time_;
    double time_granularity_;
    int steps_;
    double step_time_;
    bool pub_trajectories_;
    std::string traj_frame_;

    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr timer_;

    void computeTrajectory();
    nav_2d_msgs::msg::Path2D generateTrajectory(
        const geometry_msgs::msg::Pose2D& start_pose,
        const nav_2d_msgs::msg::Twist2D& current_vel,
        const nav_2d_msgs::msg::Twist2D& target_vel);
    nav_2d_msgs::msg::Twist2D computeVelocity(
        const nav_2d_msgs::msg::Twist2D& current_vel, 
        const nav_2d_msgs::msg::Twist2D& target_vel,
        double dt);
    geometry_msgs::msg::Pose2D computePose(
        const geometry_msgs::msg::Pose2D& start_pose,
        const nav_2d_msgs::msg::Twist2D& vel,
        double dt);
    double projectVelocity(double current_vel, double acc, double decel, double dt, double target_vel);
    void odomCB(const nav_msgs::msg::Odometry& odom)
    {
        odom_ = odom;
    }
};
}

#endif