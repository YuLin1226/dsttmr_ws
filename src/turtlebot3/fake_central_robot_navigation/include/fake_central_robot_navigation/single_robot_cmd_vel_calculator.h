#if !defined(_SINGLE_ROBOT_CMD_VEL_CALCULATOR_H_)
#define _SINGLE_ROBOT_CMD_VEL_CALCULATOR_H_

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>

namespace DSTTMR
{
    class SingleRobotCommandVelocityCalculator
    {
        public:
            SingleRobotCommandVelocityCalculator(ros::NodeHandle& private_nh, std::string topic_name);
            void publishCommandVelocity(const double& vx, const double& vy);
        private:
            ros::NodeHandle private_nh_;
            ros::Publisher cmd_vel_pub_;
            void calculateRobotCommandVelocity(const double& vx, const double& vy);
            struct TwistRobotCommand
            {
                double vx, wz;
                TwistRobotCommand(): vx(0.0), wz(0.0) {}
            }cmd_vel_;

    };
} // namespace DSTTMR

#endif // _SINGLE_ROBOT_CMD_VEL_CALCULATOR_H_


