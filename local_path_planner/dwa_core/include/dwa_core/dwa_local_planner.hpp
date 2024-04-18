#ifndef DWA_LOCAL_PLANNER_HPP_
#define DWA_LOCAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "dwa_core/xytheta_velocity_iterator.hpp"
#include "dwa_core/kinematics_parameters.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "nav_2d_msgs/msg/path2_d_list.hpp"
#include "dwa_critics/base_critic.hpp"

namespace dwa_core
{
/**
 * @class DWALocalPlanner
 * @brief A planner that uses the global plan as a guide to find an optimal local plan.
 * Samples velocity to generate local plans and scores them.
*/

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
    std::mutex odom_mtx_;
    std::string odom_topic_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    nav_msgs::msg::Path global_plan_;
    std::mutex global_plan_mtx_;
    bool global_plan_set_;

    rclcpp_lifecycle::LifecyclePublisher<nav_2d_msgs::msg::Path2DList>::SharedPtr plans_pub_;
    nav_2d_msgs::msg::Path2DList plans_;

    pluginlib::ClassLoader<dwa_critics::BaseCritic> critic_loader_;
    std::vector<dwa_critics::BaseCritic::Ptr> critics_;
    std::vector<std::string> critic_names_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    double sim_time_;
    double time_granularity_;
    int steps_;
    bool pub_plans_;
    std::string plan_frame_;

    rclcpp::Logger logger_;
    rclcpp::TimerBase::SharedPtr timer_;

    void computePlan();
    nav_2d_msgs::msg::Path2D generatePlan(
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
        std::lock_guard<std::mutex> lock(odom_mtx_);
        odom_ = odom;
    }
    void globalPlanCB(const nav_msgs::msg::Path& global_plan)
    {
        std::lock_guard<std::mutex> lock(global_plan_mtx_);
        global_plan_ = global_plan;
    }
    bool loadCritics();

    /**
     * @brief The global_plan_ variable gets updated everytime this node receives the global plan
     * from the global path planner. This function uses the global plan to generate an adjusted global plan by:
     * 1. cuting the global plan where the local costmap ends
     * 2. filling in poses in between consecutive poses to match local costmap's resolution
    */
    nav_2d_msgs::msg::Path2D prepareGlobalPlan();
};
}

#endif