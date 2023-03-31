#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <memory>
#include <thread>
#include <mutex>
#include <string>

namespace DSTTMR
{
    class FakeCentralOdom
    {
        public:
            static uint8_t SLEEP_TIME_IN_MS = 10;
            void start();
            void stop();
            FakeCentralOdom();
            ~FakeCentralOdom();

        private:
            void threadBody();
            void getFirstOdomData();
            void getSecondOdomData();
            void publishCentralOdomData();

        private:
            ros::NodeHandle private_nh_;
            tf::TransformListener first_odom_listener_, second_odom_listener_;
            std::thread first_thread_, second_thread_;
            std::mutex first_mtx_, second_mtx_;
            std::string first_base_footprint_, second_base_footprint_;
            
    }
} // namespace DSTTMR
