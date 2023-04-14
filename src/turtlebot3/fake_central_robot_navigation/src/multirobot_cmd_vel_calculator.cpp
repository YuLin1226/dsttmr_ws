#include "multirobot_cmd_vel_calculator.h"
#include <cmath>

namespace DSTTMR
{
    MultiRobot_CommandVelcity_Calculator::MultiRobot_CommandVelcity_Calculator(ros::NodeHandle private_nh)
    : private_nh_(private_nh)
    {

        first_robot_tf_listener_ = std::make_shared<TF_LISTENER>("/map", "/first_robot_base_footprint");
        second_robot_tf_listener_ = std::make_shared<TF_LISTENER>("/map", "/second_robot_base_footprint");

        first_robot_cmd_vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>("/first_robot_cmd_vel", 1000);
        second_robot_cmd_vel_pub_ = private_nh_.advertise<geometry_msgs::Twist>("/second_robot_cmd_vel", 1000);

        cmd_vel_sub_ = private_nh_.subscribe(
            "/cmd_vel", 
            1000, 
            &MultiRobot_CommandVelcity_Calculator::centralRobotCommandVelocityCallback, 
            this);
    }

    double MultiRobot_CommandVelcity_Calculator::getRobotDistance()
    {
        double dist(0.0);
        double first_robot_position_x, first_robot_position_y, first_robot_rotation_yaw;
        double second_robot_position_x, second_robot_position_y, second_robot_rotation_yaw;
        first_robot_tf_listener_->updateRobotPose(first_robot_position_x, first_robot_position_y, first_robot_rotation_yaw);
        second_robot_tf_listener_->updateRobotPose(second_robot_position_x, second_robot_position_y, second_robot_rotation_yaw);
        double dx = first_robot_position_x - second_robot_position_x;
        double dy = first_robot_position_y - second_robot_position_y;
        dist = sqrt(dx*dx + dy*dy);
        return dist;
    }

    void MultiRobot_CommandVelcity_Calculator::centralRobotCommandVelocityCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        publishTwoRobotsCommandVelocity(getRobotDistance(), cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z);
    }

    void MultiRobot_CommandVelcity_Calculator::publishTwoRobotsCommandVelocity(const double& robot_distance, const double& vx, const double& vy, const double& w)
    {
        // Use dual-steering-robot model to calculate 1st & 2nd robots' cmd_vel
        {
            double v1x = vx;
            double v1y = vy + w*robot_distance/2;
            double v2x = vx;
            double v2y = vy - w*robot_distance/2;
            // publish cmd_vel of the 1st robot
            {
                geometry_msgs::Twist first_robot_cmd_vel;
            }
            // publish cmd_vel of the 2nd robot
            {
                geometry_msgs::Twist second_robot_cmd_vel;
            }
        }
    }


} // namespace DSTTMR

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multirobot_cmd_vel_calculator");
    ros::NodeHandle private_nh("~");
    auto multirobot_cmd_calculator = std::make_shared<DSTTMR::MultiRobot_CommandVelcity_Calculator>(private_nh);
    ros::spin();
    return 0;
};
