#if !defined(_TF_LISTENER_H_)
#define _TF_LISTENER_H_

#include <ros/ros.h>
#include <string>

namespace DSTTMR
{
    class TF_LISTENER
    {
        public:
            TF_LISTENER();
            ~TF_LISTENER();
            double getDistanceBetweenTwoRobots();
        private:
            void echoRobotTF();
            voide updateRobotDistance();
            std::string first_robot_basefootprint_name, second_robot_basefootprint_name;
            double robot_distance_;
            struct RobotPose
            {
                float position_x, position_y, rotation_yaw;
                RobotPose():position_x(0.0), position_y(0.0), rotation_yaw(0.0) {}
            }first_robot_pose_, second_robot_pose_;
    };
}



#endif // _TF_LISTENER_H_
