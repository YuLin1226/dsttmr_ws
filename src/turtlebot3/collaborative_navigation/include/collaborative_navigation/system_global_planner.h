#if !defined(_SYSTEM_GLOBAL_PLANNER_H_)
#define _SYSTEM_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include "path_generator.h"
#include "tf_listener.h"
#include <memory>
#include <string>

namespace DSTTMR
{
    class SystemGlobalPlanner
    {
        private:
            ros::Publisher agents_global_planner_pub_;
            ros::NodeHandle private_nh_;

            std::string first_robot_frame_name_;
            std::string second_robot_frame_name_;
            std::shared_ptr<TF_LISTENER> first_robot_tf_listener_;
            std::shared_ptr<TF_LISTENER> second_robot_tf_listener_;

            struct RobotPose
            {
                double position_x, position_y, rotation_yaw;
                RobotPose() : position_x{0.0}, position_y{0.0}, rotation_yaw{0.0} {}
            }robot_pose_;

        public:
            SystemGlobalPlanner(ros::NodeHandle& private_nh);
            void publishGlobalPlan();
            
    };
} // namespace DSTTMR


#endif // _SYSTEM_GLOBAL_PLANNER_H_
