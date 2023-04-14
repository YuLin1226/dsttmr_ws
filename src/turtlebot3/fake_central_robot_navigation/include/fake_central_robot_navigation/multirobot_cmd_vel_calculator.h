#if !defined(_MULTIROBOT_CMD_VEL_CALCULATOR_H_)
#define _MULTIROBOT_CMD_VEL_CALCULATOR_H_

#include <ros/ros.h>
#include <memory>
#include "tf_listener.h"
#include <geometry_msgs/Twist.h>
#include "single_robot_cmd_vel_calculator.h"

namespace DSTTMR
{
    class MultiRobot_CommandVelcity_Calculator
    {
        public:
            MultiRobot_CommandVelcity_Calculator(ros::NodeHandle private_nh);
            ~MultiRobot_CommandVelcity_Calculator(){}

        private:
            ros::NodeHandle private_nh_;
            std::shared_ptr<TF_LISTENER> first_robot_tf_listener_;
            std::shared_ptr<TF_LISTENER> second_robot_tf_listener_;
            std::shared_ptr<SingleRobotCommandVelocityCalculator> first_robot_cmd_calculator_;
            std::shared_ptr<SingleRobotCommandVelocityCalculator> second_robot_cmd_calculator_;
            ros::Subscriber cmd_vel_sub_;

            void centralRobotCommandVelocityCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
            double getRobotDistance();
            void publishTwoRobotsCommandVelocity(const double& robot_distance, const double& vx, const double& vy, const double& w);

    };

} // namespace DSTTMR



#endif // _MULTIROBOT_CMD_VEL_CALCULATOR_H_
