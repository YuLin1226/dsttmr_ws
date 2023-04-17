#include "system_global_planner.h"
#include <nav_msgs/Path.h>

namespace DSTTMR
{
    SystemGlobalPlanner::SystemGlobalPlanner(ros::NodeHandle& private_nh)
    : private_nh_(private_nh)
    {
        agents_global_planner_pub_ = private_nh_.advertise<nav_msgs::Path>("/global_plan", 1);
    }

    void SystemGlobalPlanner::publishGlobalPlan()
    {
        auto& path_generator = DSTTMR::PathGenerator::getInstance();
        auto path = path_generator.createCirclePath(5, 10);
        
        /* we need to move & rotate path */

        nav_msgs::Path global_plan;
        global_plan.poses = path;
        agents_global_planner_pub_.publish(global_plan);
    }

} // namespace DSTTMR
