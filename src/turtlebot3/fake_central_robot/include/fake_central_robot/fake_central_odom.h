#if !defined(_FAKE_CENTRAL_ODOM_H_)
#define _FAKE_CENTRAL_ODOM_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <thread>
#include <mutex>
#include <string>

namespace DSTTMR
{
    class FakeCentralOdom
    {
        public:
            
            void start();
            void stop();
            FakeCentralOdom(ros::NodeHandle private_nh);
            ~FakeCentralOdom();

        private:
            void threadBody();
            void getFirstOdomData();
            void getSecondOdomData();
            void publishCentralOdomData();

        private:
            static inline const uint8_t SLEEP_TIME_IN_MS = 10;
            ros::NodeHandle private_nh_;
            // tf::TransformListener first_odom_listener_, second_odom_listener_;
            tf::TransformBroadcaster broadcaster_;
            std::thread first_thread_, second_thread_;
            std::mutex first_mtx_, second_mtx_;
            std::string first_base_footprint_, second_base_footprint_, fake_central_base_footprint_;
            std::string first_robot_odom_, fake_central_odom_;
            struct RobotPose
            {
                float position_x, position_y, rotation_yaw;
                RobotPose():position_x{0.0}, position_y{0.0}, rotation_yaw{0.0} {}
            }first_robot_pose_, second_robot_pose_, central_robot_pose_;
            
            struct Odometry
            {
                float position_x, position_y, rotation_yaw;
                RobotPose():position_x{0.0}, position_y{0.0}, rotation_yaw{0.0} {}
            }odom_to_map_;
            
            
    };
} // namespace DSTTMR

#endif // _FAKE_CENTRAL_ODOM_H_