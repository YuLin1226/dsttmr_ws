#include "multirobot_cmd_vel_calculator.h"

namespace DSTTMR
{
    MultiRobot_CommandVelcity_Calculator::MultiRobot_CommandVelcity_Calculator(ros::NodeHandle private_nh)
    : private_nh_(private_nh)
    {

        tf_listener_ = std::make_shared<TF_LISTENER>();

        first_robot_cmd_vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>("first_robot_cmd_vel", 1000);
        second_robot_cmd_vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>("second_robot_cmd_vel", 1000);

        cmd_vel_sub_ = private_nh_.subscribe(
            "cmd_vel", 
            1000, 
            &MultiRobot_CommandVelcity_Calculator::multiRobotCommandVelocityCalculateCallback, 
            this);
    }

    MultiRobot_CommandVelcity_Calculator::~MultiRobot_CommandVelcity_Calculator()
    {

    }

    void MultiRobot_CommandVelcity_Calculator::multiRobotCommandVelocityCalculateCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        auto dist = tf_listener_->getDistanceBetweenTwoRobots();
        publishTwoRobotsCommandVelocity(dist);
    }

    void MultiRobot_CommandVelcity_Calculator::publishTwoRobotsCommandVelocity(double& robot_distance)
    {
        // Use dual-steering-robot model to calculate 1st & 2nd robots' cmd_vel
        {
            
        }
        // publish
    }


} // namespace DSTTMR
