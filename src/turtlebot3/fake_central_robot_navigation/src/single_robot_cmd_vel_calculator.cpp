#include "single_robot_cmd_vel_calculator.h"

namespace DSTTMR
{
    
    SingleRobotCommandVelocityCalculator::SingleRobotCommandVelocityCalculator(ros::NodeHandle& private_nh, std::string topic_name)
    : private_nh_(private_nh)
    {
        cmd_vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>(topic_name, 1000);
    }

    void SingleRobotCommandVelocityCalculator::publishCommandVelocity(const double& vx, const double& vy)
    {
        calculateRobotCommandVelocity(vx, vy);
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = cmd_vel_.vx;
        cmd_vel.angular.z = cmd_vel_.wz;
        cmd_vel_pub_.publish(cmd_vel);
    }

    void SingleRobotCommandVelocityCalculator::calculateRobotCommandVelocity(const double& vx, const double& vy)
    {
        // cmd_vel_.vx = 0;
        // cmd_vel_.wz = 0;
    }

} // namespace DSTTMR
