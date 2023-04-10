#include "tf_listener.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <cmath>

namespace DSTTMR
{
    TF_LISTENER::TF_LISTENER()
    : robot_distance_(0.0)
    , first_robot_basefootprint_name_("/first_robot_base_footprint")
    , second_robot_basefootprint_name("/second_robot_base_footprint")
    {
    }

    TF_LISTENER::~TF_LISTENER()
    {
    }

    double TF_LISTENER::getDistanceBetweenTwoRobots()
    {
        echoRobotTF();
        updateRobotDistance();
        return robot_distance_;
    }

    void TF_LISTENER::echoRobotTF()
    {
        tf::StampedTransform transform;
        tf::TransformListener listener;
        try
        {
            {
                listener.lookupTransform("/map", first_robot_basefootprint_name_, 
                                        ros::Time(0), transform);
                first_robot_pose_.position_x = transform.getOrigin().x();
                first_robot_pose_.position_y = transform.getOrigin().y();
                tf::Quaternion q = transform.getRotation(); 
                first_robot_pose_.rotation_yaw = tf::getYaw(q);
            }
            {
                listener.lookupTransform("/map", second_robot_basefootprint_name, 
                                        ros::Time(0), transform);
                second_robot_pose_.position_x = transform.getOrigin().x();
                second_robot_pose_.position_y = transform.getOrigin().y();
                tf::Quaternion q = transform.getRotation(); 
                second_robot_pose_.rotation_yaw = tf::getYaw(q);
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    void TF_LISTENER::updateRobotDistance()
    {
        double dx = first_robot_pose_.position_x - second_robot_pose_.position_y;
        double dy = first_robot_pose_.position_y - second_robot_pose_.position_y;
        robot_distance_ = sqrt(dx*dx + dy*dy);
    }
    
} // namespace DSTTMR
