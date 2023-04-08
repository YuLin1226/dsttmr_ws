#include "fake_central_odom.h"
#include <cmath>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace DSTTMR
{
    FakeCentralOdom::FakeCentralOdom(ros::NodeHandle private_nh)
    : private_nh_(private_nh)
    , fake_central_base_footprint_("/fake_central_base_footprint")
    , first_base_footprint_("/first_robot_base_footprint")
    , second_base_footprint_("/second_robot_base_footprint")
    , first_robot_odom_("/first_robot_odom")
    , fake_central_odom_("/odom")
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
        tf::StampedTransform transform;
        tf::TransformListener listener;
        while (private_nh_.ok())
        {
            try
            {
                {
                    listener.lookupTransform(first_robot_odom_, "/map",  
                                            ros::Time(0), transform);
                    odom_to_map_.position_x = transform.getOrigin().x();
                    odom_to_map_.position_y = transform.getOrigin().y();
                    tf::Quaternion q = transform.getRotation(); 
                    odom_to_map_.rotation_yaw = tf::getYaw(q);
                }
                {
                    listener.lookupTransform(first_base_footprint_, "/map",  
                                            ros::Time(0), transform);
                    first_robot_pose_.position_x = transform.getOrigin().x();
                    first_robot_pose_.position_y = transform.getOrigin().y();
                    tf::Quaternion q = transform.getRotation(); 
                    first_robot_pose_.rotation_yaw = tf::getYaw(q);
                }
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
        tf::StampedTransform transform;
        tf::TransformListener listener;
        while (private_nh_.ok())
        {
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
        {
            // compute central robot position
            central_robot_pose_.position_x = (first_robot_pose_.position_x + second_robot_pose_.position_x) / 2.0;
            central_robot_pose_.position_y = (first_robot_pose_.position_y + second_robot_pose_.position_y) / 2.0;
            // compute central robot yaw: atan2(dy, dx)
            float dx = first_robot_pose_.position_x - second_robot_pose_.position_x;
            float dy = first_robot_pose_.position_y - second_robot_pose_.position_y;
            central_robot_pose_.rotation_yaw = atan2(dy, dx);
        }
        // map -> odom
        {
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(odom_to_map_.position_x, odom_to_map_.position_y, 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, odom_to_map_.rotation_yaw);
            transform.setRotation(q);
            broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", fake_central_odom_));
        }
        // odom -> base_footprint
        {
            /*
            use eigen lib to compute odom->base_footprint:
            T = | c -s tx | is map->odom
                | s  c ty |
                | 0  0  1 |
            odom->base_footprint = inv(T) * map->base_footprint
            */
            MatrixXd T_odom2map(3, 3);
            T_odom2map(0, 0) = cos(odom_to_map_.rotation_yaw);
            T_odom2map(0, 1) = - sin(odom_to_map_.rotation_yaw);
            T_odom2map(1, 0) = cos(odom_to_map_.rotation_yaw);
            T_odom2map(1, 1) = sin(odom_to_map_.rotation_yaw);
            T_odom2map(0, 2) = odom_to_map_.position_x;
            T_odom2map(1, 2) = odom_to_map_.position_y;
            T_odom2map(2, 2) = 1;
            Vector3d V_basefootprint2map, V_basefootprint2odom;
            V_basefootprint2map(0) = central_robot_pose_.position_x;
            V_basefootprint2map(1) = central_robot_pose_.position_y;
            V_basefootprint2map(2) = 1;
            V_basefootprint2odom = T_odom2map.inverse() * V_basefootprint2map;

            tf::Transform transform;
            transform.setOrigin( tf::Vector3(V_basefootprint2odom(0), V_basefootprint2odom(1), 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, odom_to_map_.rotation_yaw - central_robot_pose_.rotation_yaw);
            transform.setRotation(q);
            broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fake_central_odom_, fake_central_base_footprint_));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

} // namespace DSTTMR


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_central_odom_node");
    ros::NodeHandle nh, private_nh("~");

    auto fake_central_odom_ptr = std::make_shared<DSTTMR::FakeCentralOdom>(private_nh);
    fake_central_odom_ptr->start();
    return 0;
};