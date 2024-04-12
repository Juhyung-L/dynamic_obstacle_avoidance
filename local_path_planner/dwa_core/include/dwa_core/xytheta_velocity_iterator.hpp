#ifndef XYTHETA_VELOCITY_ITERATOR_HPP_
#define XYTHETA_VELOCITY_ITERATOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "dwa_core/one_d_velocity_iterator.hpp"
#include "dwa_core/kinematics_parameters.hpp"

namespace dwa_core
{
/**
 * @class XYThetaVelocityIterator
 * @brief A wrapper class for 3 OneDVelocityIterators (x, y, theta)
*/

class XYThetaVelocityIterator
{
public:
    XYThetaVelocityIterator()
    {}

    /**
     * @param num_iterations the valid velocity ranges will be divided by this number.
     * So the maximum number of iterations for each dimension is num_iterations+1
     * and the maximum total number of iterations is (num_iterations+1)^3
    */
    XYThetaVelocityIterator(int num_iterations, const KinematicsParameters::SharedPtr& kp)
    : num_iterations_(num_iterations)
    , kp_(kp)
    {}

    void initialize(const nav_2d_msgs::msg::Twist2D& current_vel, double dt)
    {
        is_finished_ = false;
        x_it_.initialize(current_vel.x, kp_->getMinVelX(), kp_->getMaxVelX(),
            kp_->getAccX(), kp_->getDecelX(), dt, num_iterations_
        );
        y_it_.initialize(current_vel.y, kp_->getMinVelY(), kp_->getMaxVelY(),
            kp_->getAccY(), kp_->getDecelY(), dt, num_iterations_
        );
        theta_it_.initialize(current_vel.theta, kp_->getMinVelTheta(), kp_->getMaxVelTheta(),
            kp_->getAccTheta(), kp_->getDecelTheta(), dt, num_iterations_
        );
    }

    XYThetaVelocityIterator& operator++()
    {
        if (!x_it_.isFinished())
        {
            ++x_it_;
            return *this;
        }

        if (!y_it_.isFinished())
        {
            x_it_.reset();
            ++y_it_;
            return *this;
        }

        if (!theta_it_.isFinished())
        {
            x_it_.reset();
            y_it_.reset();
            ++theta_it_;
            return *this;
        }

        is_finished_ = true;
        return *this;
    }

    bool isFinished()
    {
        return is_finished_;
    }

    nav_2d_msgs::msg::Twist2D getCurrentVel()
    {
        nav_2d_msgs::msg::Twist2D twist;
        twist.x = x_it_.getCurrentVelocity();
        twist.y = y_it_.getCurrentVelocity();
        twist.theta = theta_it_.getCurrentVelocity();
        return twist;
    }

private:
    int num_iterations_;
    bool is_finished_;

    KinematicsParameters::SharedPtr kp_;

    OneDVelocityIterator x_it_;
    OneDVelocityIterator y_it_;
    OneDVelocityIterator theta_it_;
};
}

#endif