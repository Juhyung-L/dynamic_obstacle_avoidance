#include <chrono>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "dwa_core/dwa_local_planner.hpp"
#include "dwa_utils/conversions.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwa_utils/transform_utils.hpp"
#include "dwa_critics/base_critic.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace dwa_core
{
DWALocalPlanner::DWALocalPlanner(rclcpp::NodeOptions node_options)
: nav2_util::LifecycleNode("dwa_local_planner", "", node_options)
, critic_loader_("dwa_critics", "dwa_critics::BaseCritic")
, logger_(this->get_logger())
{
    this->declare_parameter("num_iterations", rclcpp::ParameterValue(1));
    this->declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
    this->declare_parameter("sim_time", rclcpp::ParameterValue(3.0));
    this->declare_parameter("time_granularity", rclcpp::ParameterValue(0.5));
    this->declare_parameter("debug", rclcpp::ParameterValue(false));
    this->declare_parameter("critic_names", rclcpp::PARAMETER_STRING_ARRAY);

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    sim_time_ = this->get_parameter("sim_time").as_double();
    time_granularity_ = this->get_parameter("time_granularity").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    critic_names_ = this->get_parameter("critic_names").as_string_array();
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "local_costmap", std::string{get_namespace()}, "local_costmap"
    );
    costmap_frame_ = "odom";
    trajs_.header.frame_id = costmap_frame_;
    steps_ = std::ceil(sim_time_ / time_granularity_);
    global_traj_set_ = false;
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
    global_traj_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "plan", rclcpp::SystemDefaultsQoS(), std::bind(&DWALocalPlanner::globalTrajCB, this, _1)
    );
    trajs_pub_ = this->create_publisher<nav_2d_msgs::msg::Path2DList>(
        "sample_trajectories", rclcpp::SystemDefaultsQoS()
    );
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rclcpp::SystemDefaultsQoS()
    );
    global_traj_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "global_trajectory", rclcpp::SystemDefaultsQoS()
    );

    kp_ = std::make_shared<KinematicsParameters>(shared_from_this());
    vel_it_ = XYThetaVelocityIterator(shared_from_this(), kp_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (!loadCritics())
    {
        return nav2_util::CallbackReturn::FAILURE;
    }
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWALocalPlanner::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Activating");

    trajs_pub_->on_activate();
    cmd_vel_pub_->on_activate();
    global_traj_pub_->on_activate();
    costmap_ros_->activate();

    // 2 Hz timer
    timer_ = this->create_wall_timer(
        250ms, std::bind(&DWALocalPlanner::computeVelocityCommand, this)
    );

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWALocalPlanner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Deactivating");

    global_traj_set_ = false;
    trajs_pub_->on_deactivate();
    cmd_vel_pub_->on_deactivate();
    global_traj_pub_->on_deactivate();
    timer_->cancel();
    if (costmap_ros_->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        costmap_ros_->deactivate();
    }

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
    cmd_vel_pub_.reset();
    global_traj_pub_.reset();
    timer_.reset();
    tf_buffer_.reset();
    tf_listener_.reset();
    for (auto& critic : critics_)
    {
        critic.reset();
    }

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

void DWALocalPlanner::computeVelocityCommand()
{
    if (!global_traj_set_)
    {
        return;
    }
    
    // lock costmap
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    // get robot pose
    geometry_msgs::msg::PoseStamped pose;
    if (!costmap_ros_->getRobotPose(pose)) // this gets robot pose in odom frame
    {
        RCLCPP_ERROR(logger_, "Failed to get robot pose");
        return;
    }

    // prepare global trajectory
    std::unique_lock<std::mutex> global_traj_lock(global_traj_mtx_);
    nav_2d_msgs::msg::Path2D global_traj_2d = dwa_utils::path3Dto2D(global_traj_);
    global_traj_lock.unlock();

    nav_2d_msgs::msg::Path2D adjusted_global_traj = prepareGlobalTrajectory(global_traj_2d);
    nav_2d_msgs::msg::Path2D transformed_global_traj;
    if (!dwa_utils::transformPath2D(tf_buffer_, costmap_frame_, adjusted_global_traj, transformed_global_traj))
    {
        return;
    }
    transformed_global_traj.header.frame_id = costmap_frame_;

    for (auto& critic : critics_)
    {
        critic->prepare(transformed_global_traj);
    }

    if (debug_)
    {
        global_traj_pub_->publish(dwa_utils::path2Dto3D(transformed_global_traj));
    }

    trajs_.paths.clear();
    double best_score = std::numeric_limits<double>::min();
    nav_2d_msgs::msg::Path2D best_traj;
    nav_2d_msgs::msg::Twist2D best_cmd_vel;
    geometry_msgs::msg::Pose2D start_pose = dwa_utils::pose3Dto2D(pose.pose);

    // lock to read from odom_
    std::unique_lock<std::mutex> odom_lock(odom_mtx_);
    nav_2d_msgs::msg::Twist2D current_vel = dwa_utils::twist3Dto2D(odom_.twist.twist);
    odom_lock.unlock();

    // sample local trajs and score them
    vel_it_.initialize(current_vel, sim_time_);
    while (!vel_it_.isFinished())
    {
        double score = 0.0;

        // generate trajs
        nav_2d_msgs::msg::Twist2D cmd_vel = vel_it_.getCurrentVel();
        nav_2d_msgs::msg::Path2D traj = generateTrajectory(start_pose, current_vel, cmd_vel);

        // store trajs for rviz
        if (debug_)
        {
            trajs_.paths.push_back(traj);
        }
        
        // score trajs
        for (auto& critic : critics_)
        {
            score += critic->scoreTrajectory(traj);
        }

        // store the best trajectory
        if (score > best_score)
        {
            best_score = score;
            best_traj = traj;
            best_cmd_vel = cmd_vel;
        }

        // increment velocity iterator
        ++vel_it_;
    }

    cmd_vel_pub_->publish(dwa_utils::twist2Dto3D(best_cmd_vel));

    if (debug_)
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
    for (int i=1; i<steps_; ++i)
    {
        vel = computeVelocity(vel, target_vel, time_granularity_);
        pose = computePose(pose, vel, time_granularity_);
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
    new_pose.x = start_pose.x + (vel.x * cos(start_pose.theta) - vel.y * sin(start_pose.theta)) * dt;
    new_pose.y = start_pose.y + (vel.x * sin(start_pose.theta) + vel.y * cos(start_pose.theta)) * dt;
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

nav_2d_msgs::msg::Path2D DWALocalPlanner::prepareGlobalTrajectory(nav_2d_msgs::msg::Path2D in_traj)
{
    nav_2d_msgs::msg::Path2D out_traj;
    out_traj.header.frame_id = in_traj.header.frame_id;

    if (in_traj.poses.empty())
    {
        return out_traj;
    }

    // cut the global trajectory where the local costmap ends
    // if the global trajectory ends inside of the costmap, it doesn't get cut
    unsigned int x, y;
    int i = 0;
    for (; i<in_traj.poses.size(); ++i)
    {
        // the first pose that is outside of the local costmap
        if (!costmap_->worldToMap(in_traj.poses[i].x, in_traj.poses[i].y, x, y))
        {
            break;
        }
    }
    in_traj.poses.resize(i);

    // interpolate
    geometry_msgs::msg::Pose2D prev = in_traj.poses[0];
    out_traj.poses.push_back(prev);
    double resolution = costmap_->getResolution();
    for (int i=0; i<in_traj.poses.size(); ++i)
    {
        geometry_msgs::msg::Pose2D cur;
        cur.x = in_traj.poses[i].x;
        cur.y = in_traj.poses[i].y;
        double dx = (cur.x - prev.x);
        double dy = (cur.y - prev.y);
        double dtheta = (cur.theta - prev.theta);
        double dist = 
            std::sqrt(dx*dx + dy*dy);
        if (dist > resolution)
        {
            int steps = static_cast<int>(dist / resolution);
            double step_x = dx / steps;
            double step_y = dy / steps;
            double step_theta = dtheta / steps;
            for (int j=1; j<steps; ++j)
            {
                geometry_msgs::msg::Pose2D interpolated_pose;
                interpolated_pose.x = prev.x + j* step_x;
                interpolated_pose.y = prev.y + j* step_y;
                interpolated_pose.theta = step_theta + j * step_theta;
                out_traj.poses.push_back(interpolated_pose);
            }
        }
        out_traj.poses.push_back(cur);
        prev = cur;
    }

    // when cutting the global trajectory, the last pose is the first pose outside of the costmap
    // then we interpolated
    // so we need to cut the trajectory once more to get rid of interpolated poses outside of the costmap
    i = 0;
    for (; i<out_traj.poses.size(); ++i)
    {
        if (!costmap_->worldToMap(out_traj.poses[i].x, out_traj.poses[i].y, x, y))
        {
            break;
        }
    }
    out_traj.poses.resize(i);
    return out_traj;
}

bool DWALocalPlanner::loadCritics()
{
    if (critic_names_.empty())
    {
        RCLCPP_ERROR(logger_, "No critics defined");
        return false;
    }

    for (auto critic_name : critic_names_)
    {
        critic_name = "dwa_critics::" + critic_name + "Critic";
        dwa_critics::BaseCritic::Ptr critic = critic_loader_.createUniqueInstance(critic_name);
        try
        {
            critic->initialize(shared_from_this(), costmap_ros_);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "Critic %s failed to initialize", critic_name.c_str());
            return false;
        }
        critics_.push_back(critic);
    }
    return true;
}
}