#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "fake_central_odom.h"

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
    }

    void FakeCentralOdom::getFirstOdomData()
    {
        while (private_nh_.ok())
        {
            tf::StampedTransform transform;
            try
            {
                listener.lookupTransform(first_base_footprint_, "/map",  
                                        ros::Time(0), transform);
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
            try
            {
                listener.lookupTransform(second_base_footprint_, "/map",  
                                        ros::Time(0), transform);
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
        
    }

} // namespace DSTTMR


// int main(int argc, char** argv)
// {
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
//     return 0;
// };