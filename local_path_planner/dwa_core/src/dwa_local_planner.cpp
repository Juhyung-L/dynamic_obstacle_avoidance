#include <chrono>
#include <vector>

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
    this->declare_parameter("pub_planectories", rclcpp::ParameterValue(false));
    this->declare_parameter("plan_frame", rclcpp::ParameterValue("odom"));
    this->declare_parameter("critic_names", rclcpp::PARAMETER_STRING_ARRAY);

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    sim_time_ = this->get_parameter("sim_time").as_double();
    time_granularity_ = this->get_parameter("time_granularity").as_double();
    pub_plans_ = this->get_parameter("pub_planectories").as_bool();
    plan_frame_ = this->get_parameter("plan_frame").as_string();
    critic_names_ = this->get_parameter("critic_names").as_string_array();

    plans_.header.frame_id = plan_frame_;
    steps_ = std::ceil(sim_time_ / time_granularity_);
    global_plan_set_ = false;

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
    global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "plan", rclcpp::SystemDefaultsQoS(), std::bind(&DWALocalPlanner::globalPlanCB, this, _1)
    );
    plans_pub_ = this->create_publisher<nav_2d_msgs::msg::Path2DList>(
        "dwa_planectories", rclcpp::SystemDefaultsQoS()
    );

    kp_ = std::make_shared<KinematicsParameters>(shared_from_this());
    vel_it_ = XYThetaVelocityIterator(shared_from_this(), kp_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

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

    plans_pub_->on_activate();
    costmap_ros_->activate();

    // 2 Hz timer
    timer_ = this->create_wall_timer(
        500ms, std::bind(&DWALocalPlanner::computePlan, this)
    );

    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
DWALocalPlanner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(logger_, "Deactivating");

    global_plan_set_ = false;
    plans_pub_->on_deactivate();
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
    plans_pub_.reset();
    timer_.reset();
    tf_buffer_.reset();
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

void DWALocalPlanner::computePlan()
{
    if (!global_plan_set_)
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

    // prepare global plan
    std::unique_lock<std::mutex> global_plan_lock(global_plan_mtx_);
    nav_2d_msgs::msg::Path2D adjusted_global_plan = prepareGlobalPlan();
    nav_2d_msgs::msg::Path2D transformed_global_plan;
    dwa_utils::transformPath2D(
        tf_buffer_,plan_frame_, adjusted_global_plan, transformed_global_plan, this->get_clock(), rclcpp::Duration(30ms));
    global_plan_lock.lock();

    // set global plan in the critics


    // sample local plans and score them
    plans_.paths.clear();
    double best_score = DBL_MIN;
    nav_2d_msgs::msg::Path2D best_plan;
    nav_2d_msgs::msg::Twist2D best_target_vel;
    geometry_msgs::msg::Pose2D start_pose = dwa_utils::pose3Dto2D(pose.pose);

    // lock to read from odom_
    std::unique_lock<std::mutex> odom_lock(odom_mtx_);
    nav_2d_msgs::msg::Twist2D current_vel = dwa_utils::twist3Dto2D(odom_.twist.twist);
    odom_lock.unlock();

    vel_it_.initialize(current_vel, sim_time_);
    while (!vel_it_.isFinished())
    {
        double score = 0.0;

        // generate plans
        nav_2d_msgs::msg::Twist2D target_vel = vel_it_.getCurrentVel();
        nav_2d_msgs::msg::Path2D plan = generatePlan(start_pose, current_vel, target_vel);

        // store plans for rviz
        if (pub_plans_)
        {
            plans_.paths.push_back(plan);
        }

        // score plans
        for (auto& critic : critics_)
        {
            score += critic->scorePlan(plan);
        }

        // store the best plan
        if (score > best_score)
        {
            best_plan = plan;
            best_target_vel = target_vel;
        }

        // increment velocity iterator
        ++vel_it_;
    }



    if (pub_plans_)
    {
        plans_pub_->publish(plans_);
    }
}

nav_2d_msgs::msg::Path2D DWALocalPlanner::generatePlan(
    const geometry_msgs::msg::Pose2D& start_pose,
    const nav_2d_msgs::msg::Twist2D& current_vel,
    const nav_2d_msgs::msg::Twist2D& target_vel)
{
    nav_2d_msgs::msg::Path2D plan;
    plan.poses.push_back(start_pose); // push the start pose
    geometry_msgs::msg::Pose2D pose = start_pose;
    nav_2d_msgs::msg::Twist2D vel = current_vel;
    for (int i=1; i<steps_; ++i)
    {
        vel = computeVelocity(vel, target_vel, time_granularity_);
        pose = computePose(pose, vel, time_granularity_);
        plan.poses.push_back(pose);
    }
    return plan;
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

nav_2d_msgs::msg::Path2D DWALocalPlanner::prepareGlobalPlan()
{
    nav_2d_msgs::msg::Path2D cut_plan;
    nav_2d_msgs::msg::Path2D interpolated_plan;

    if (global_plan_.poses.empty())
    {
        return cut_plan;
    }

    // cut the global plan where the local costmap ends
    // if the global plan ends inside of the costmap, it doesn't get cut
    geometry_msgs::msg::Pose2D pose_2d;
    unsigned int x, y;
    for (auto& pose : global_plan_.poses)
    {
        pose_2d.x = pose.pose.position.x;
        pose_2d.y = pose.pose.position.y;
        cut_plan.poses.push_back(pose_2d);

        // the first pose that is outside of the local costmap
        if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.x, x, y))
        {
            break;
        }
    }

    // interpolate
    geometry_msgs::msg::Pose2D prev = cut_plan.poses[0];
    interpolated_plan.poses.push_back(prev);
    double resolution = costmap_->getResolution();
    for (int i=0; i<cut_plan.poses.size(); ++i)
    {
        geometry_msgs::msg::Pose2D cur;
        cur.x = cut_plan.poses[i].x;
        cur.y = cut_plan.poses[i].y;
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
                interpolated_plan.poses.push_back(interpolated_pose);
            }
        }
        interpolated_plan.poses.push_back(cur);
        prev = cur;
    }

    // when cutting the global plan, the last pose is the first pose outside of the costmap
    // then we interpolated
    // so we need to cut the plan once more to get rid of interpolated poses outside of the costmap
    int i = 0;
    for (; interpolated_plan.poses.size(); ++i)
    {
        if (!costmap_->worldToMap(interpolated_plan.poses[i].x, interpolated_plan.poses[i].y, x, y))
        {
            break;
        }
    }
    interpolated_plan.poses.erase(interpolated_plan.poses.begin() + i + 1, interpolated_plan.poses.end());
    return interpolated_plan;
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