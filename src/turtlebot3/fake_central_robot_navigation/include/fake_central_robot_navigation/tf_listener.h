#if !defined(_TF_LISTENER_H_)
#define _TF_LISTENER_H_

#include <ros/ros.h>
#include <string>

namespace DSTTMR
{
    class TF_LISTENER
    {
        public:
            TF_LISTENER(std::string parent_frame_name, std::string child_frame_name);
            ~TF_LISTENER() {}
            void updateRobotPose(double& robot_position_x, double& robot_position_y, double& rotation_yaw);
        private:
            void echoRobotTF();
            std::string parent_frame_name_, child_frame_name_;
            double position_x_, position_y_, rotation_yaw_;
    };
}



#endif // _TF_LISTENER_H_
