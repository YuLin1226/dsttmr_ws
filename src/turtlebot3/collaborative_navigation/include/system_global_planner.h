#if !defined(_SYSTEM_GLOBAL_PLANNER_H_)
#define _SYSTEM_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include "path_generator.h"
#include "collaborative_navigation/System_global_plan.h"

namespace DSTTMR
{
    class SystemGlobalPlanner
    {
        private:
            ros::Publisher agents_global_planner_pub_;
            ros::NodeHandle private_nh_;
        public:
            SystemGlobalPlanner(ros::NodeHandle& private_nh);
            void publishGlobalPlan();
            
    }
} // namespace DSTTMR


#endif // _SYSTEM_GLOBAL_PLANNER_H_
