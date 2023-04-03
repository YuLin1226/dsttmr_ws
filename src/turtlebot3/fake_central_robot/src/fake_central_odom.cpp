#include "fake_central_odom.h"
#include <cmath>
#include <tf/tf.h>

namespace DSTTMR
{
    FakeCentralOdom::FakeCentralOdom()    
    {
        // retrive ros setting & parameters
    }

    FakeCentralOdom::~FakeCentralOdom()
    {
        stop();
    }

    void FakeCentralOdom::start()
    {
        threadBody();
    }

    void FakeCentralOdom::stop()
    {
        first_thread_.join();
        second_thread_.join();
    }

    void FakeCentralOdom::threadBody()
    {
        first_thread_ = std::thread([this]()
        {
            getFirstOdomData();
        });

        second_thread_ = std::thread([this]()
        {
            getSecondOdomData();
        });
        
        // main thread
        while(private_nh_.ok())
        {
            publishCentralOdomData();
        }
    }

    void FakeCentralOdom::getFirstOdomData()
    {
        while (private_nh_.ok())
        {
            tf::StampedTransform transform;
            tf::TransformListener listener;
            try
            {
                listener.lookupTransform(first_base_footprint_, "/map",  
                                        ros::Time(0), transform);
                first_robot_pose_.position_x = transform.getOrigin().x();
                first_robot_pose_.position_y = transform.getOrigin().y();
                tf::Quaternion q = transform.getRotation(); 
                first_robot_pose_.rotation_yaw = tf::getYaw(q);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_IN_MS));
        }
    }

    void FakeCentralOdom::getSecondOdomData()
    {
        while (private_nh_.ok())
        {
            tf::StampedTransform transform;
            tf::TransformListener listener;
            try
            {
                listener.lookupTransform(second_base_footprint_, "/map",  
                                        ros::Time(0), transform);
                second_robot_pose_.position_x = transform.getOrigin().x();
                second_robot_pose_.position_y = transform.getOrigin().y();
                tf::Quaternion q = transform.getRotation(); 
                second_robot_pose_.rotation_yaw = tf::getYaw(q);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME_IN_MS));
        }
    }

    void FakeCentralOdom::publishCentralOdomData()
    {
        // compute central robot position
        central_robot_pose_.position_x = (first_robot_pose_.position_x + second_robot_pose_.position_x) / 2.0;
        central_robot_pose_.position_y = (first_robot_pose_.position_y + second_robot_pose_.position_y) / 2.0;
        // compute central robot yaw: atan2(dy, dx)
        float dx = first_robot_pose_.position_x - second_robot_pose_.position_x;
        float dy = first_robot_pose_.position_y - second_robot_pose_.position_y;
        central_robot_pose_.rotation_yaw = atan2(dy, dx);
        // publish data
    }

} // namespace DSTTMR


int main(int argc, char** argv)
{
//     ros::init(argc, argv, "my_tf_listener");
//     ros::NodeHandle node;
//     tf::TransformListener listener;

//     ros::Rate rate(10.0);
//     while (node.ok())
//     {
//         tf::StampedTransform transform;
//         try
//         {
//             listener.lookupTransform("/turtle2", "/turtle1",  
//                                     ros::Time(0), transform);
//         }
//         catch (tf::TransformException ex)
//         {
//             ROS_ERROR("%s",ex.what());
//             ros::Duration(1.0).sleep();
//         }
//         // publish central odom tf

//         rate.sleep();
//     }
    return 0;
};