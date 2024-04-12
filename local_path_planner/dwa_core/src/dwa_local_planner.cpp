#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "dwa_core/dwa_local_planner.hpp"
#include "dwa_utils/conversions.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace dwa_core
{
DWALocalPlanner::DWALocalPlanner(rclcpp::NodeOptions node_options)
: nav2_util::LifecycleNode("dwa_local_planner", "", node_options)
, logger_(this->get_logger())
{
    this->declare_parameter("num_iterations", rclcpp::ParameterValue(1));
    this->declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
    this->declare_parameter("sim_time", rclcpp::ParameterValue(3.0));
    this->declare_parameter("time_granularity", rclcpp::ParameterValue(0.5));
    this->declare_parameter("pub_trajectories", rclcpp::ParameterValue(false));
    this->declare_parameter("traj_frame", rclcpp::ParameterValue("odom"));

    num_iterations_ = this->get_parameter("num_iterations").as_int();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    sim_time_ = this->get_parameter("sim_time").as_double();
    time_granularity_ = this->get_parameter("time_granularity").as_double();
    pub_trajectories_ = this->get_parameter("pub_trajectories").as_bool();
    traj_frame_ = this->get_parameter("traj_frame").as_string();

    trajs_.header.frame_id = traj_frame_;

    steps_ = std::ceil(sim_time_ / time_granularity_);
    step_time_ = sim_time_ / steps_;

    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "local_costmap", std::string{get_namespace()}, "local_costmap"
    );
}

DWALocalPlanner::~DWALocalPlanner()
{
    costmap_thread_.reset();
}

nav2_util::CallbackReturn
DWALocalPlanner::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Configuring");
    
    costmap_ros_->configure();
    costmap_ = costmap_ros_->getCostmap();
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&DWALocalPlanner::odomCB, this, _1)
    );
    trajs_pub_ = this->create_publisher<nav_2d_msgs::msg::Path2DList>(
        "dwa_trajectories", rclcpp::SystemDefaultsQoS()
    );

    kp_ = std::make_shared<KinematicsParameters>(shared_from_this());
    vel_it_ = XYThetaVelocityIterator(num_iterations_, kp_);
    
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWALocalPlanner::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Activating");

    trajs_pub_->on_activate();
    costmap_ros_->activate();

    // 2 Hz timer
    timer_ = this->create_wall_timer(
        500ms, std::bind(&DWALocalPlanner::computeTrajectory, this)
    );

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWALocalPlanner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Deactivating");

    trajs_pub_->on_deactivate();
    costmap_ros_->deactivate();
    timer_->cancel();

    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWALocalPlanner::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Cleaning up");

    kp_.reset();
    odom_sub_.reset();
    trajs_pub_.reset();
    timer_.reset();

    if (costmap_ros_->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        costmap_ros_->cleanup();
    }
    costmap_thread_.reset();
    costmap_ = nullptr;
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWALocalPlanner::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(logger_, "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}

void DWALocalPlanner::computeTrajectory()
{
    // lock costmap
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    geometry_msgs::msg::PoseStamped pose;
    if (!costmap_ros_->getRobotPose(pose)) // this gets robot pose in odom frame
    {
        RCLCPP_ERROR(logger_, "Failed to get robot pose");
        return;
    }
    
    trajs_.paths.clear();
    nav_2d_msgs::msg::Path2D traj;
    nav_2d_msgs::msg::Twist2D target_vel;
    nav_2d_msgs::msg::Twist2D current_vel = dwa_utils::twist3Dto2D(odom_.twist.twist);
    geometry_msgs::msg::Pose2D start_pose = dwa_utils::pose3Dto2D(pose.pose);
    vel_it_.initialize(current_vel, sim_time_);
    while (!vel_it_.isFinished())
    {
        // generate trajectories
        target_vel = vel_it_.getCurrentVel();
        traj = generateTrajectory(start_pose, current_vel, target_vel);

        // store trajectories to trajectory list for rviz
        if (pub_trajectories_)
        {
            trajs_.paths.push_back(traj);
        }

        // score them

        // increment velocity iterator
        ++vel_it_;
    }

    if (pub_trajectories_)
    {
        trajs_pub_->publish(trajs_);
    }
}

nav_2d_msgs::msg::Path2D DWALocalPlanner::generateTrajectory(
    const geometry_msgs::msg::Pose2D& start_pose,
    const nav_2d_msgs::msg::Twist2D& current_vel,
    const nav_2d_msgs::msg::Twist2D& target_vel)
{
    nav_2d_msgs::msg::Path2D traj;
    traj.poses.push_back(start_pose); // push the start pose
    geometry_msgs::msg::Pose2D pose = start_pose;
    nav_2d_msgs::msg::Twist2D vel = current_vel;
    for (int i=0; i<steps_; ++i)
    {
        vel = computeVelocity(vel, target_vel, step_time_);
        pose = computePose(pose, vel, step_time_);
        traj.poses.push_back(pose);
    }
    return traj;
}

geometry_msgs::msg::Pose2D DWALocalPlanner::computePose(
    const geometry_msgs::msg::Pose2D& start_pose,
    const nav_2d_msgs::msg::Twist2D& vel,
    double dt)
{
    geometry_msgs::msg::Pose2D new_pose;
    new_pose.x = start_pose.x + (vel.x * cos(start_pose.theta) + vel.y * cos(M_PI_2 + start_pose.theta)) * dt;
    new_pose.y = start_pose.y + (vel.x * sin(start_pose.theta) + vel.y * sin(M_PI_2 + start_pose.theta)) * dt;
    new_pose.theta = start_pose.theta + vel.theta * dt;
    return new_pose;
}

nav_2d_msgs::msg::Twist2D DWALocalPlanner::computeVelocity(
    const nav_2d_msgs::msg::Twist2D& current_vel, 
    const nav_2d_msgs::msg::Twist2D& target_vel,
    double dt)
{
    nav_2d_msgs::msg::Twist2D new_vel;
    new_vel.x = projectVelocity(current_vel.x, kp_->getAccX(), kp_->getDecelX(), dt, target_vel.x);
    new_vel.y = projectVelocity(current_vel.y, kp_->getAccY(), kp_->getDecelY(), dt, target_vel.y);
    new_vel.theta = projectVelocity(current_vel.theta, kp_->getAccTheta(), kp_->getDecelTheta(), dt, target_vel.theta);
    return new_vel;
}

double DWALocalPlanner::projectVelocity(double current_vel, double acc, double decel, double dt, double target_vel)
{
    if (current_vel < target_vel)
    {
        current_vel = current_vel + acc * dt;
        return std::min(current_vel, target_vel);
    }
    else
    {
        current_vel = current_vel + decel * dt;
        return std::max(current_vel, target_vel);
    }
}
}