#include "tf_listener.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>

namespace DSTTMR
{
    TF_LISTENER::TF_LISTENER(std::string parent_frame_name, std::string child_frame_name)
    : parent_frame_name_(parent_frame_name)
    , child_frame_name_(child_frame_name)
    , position_x_(0.0)
    , position_y_(0.0)
    , rotation_yaw_(0.0)
    {
    }

    void TF_LISTENER::updateRobotPose(double& robot_position_x, double& robot_position_y, double& rotation_yaw)
    {
        echoRobotTF();
        robot_position_x = position_x_;
        robot_position_y = position_y_;
        rotation_yaw = rotation_yaw_;
    }
    void TF_LISTENER::echoRobotTF()
    {
        tf::StampedTransform transform;
        tf::TransformListener listener;
        try
        {
            {
                listener.lookupTransform(parent_frame_name_, child_frame_name_, 
                                        ros::Time(0), transform);
                position_x_ = transform.getOrigin().x();
                position_y_ = transform.getOrigin().y();
                tf::Quaternion q = transform.getRotation(); 
                rotation_yaw_ = tf::getYaw(q);
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    
} // namespace DSTTMR
