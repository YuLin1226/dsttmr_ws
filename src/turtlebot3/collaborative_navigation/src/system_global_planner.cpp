#include "system_global_planner.h"

namespace DSTTMR
{
    SystemGlobalPlanner::SystemGlobalPlanner(ros::NodeHandle& private_nh)
    : private_nh_(private_nh)
    {
        // agents_global_planner_pub_ = private_nh_.advertise<collaborative_navigation::System_global_plan>("/global_plan", 1);
    }

    void SystemGlobalPlanner::publishGlobalPlan()
    {
        auto path_generator = DSTTMR::PathGenerator::getInstance();
        auto path = path_generator.createCirclePath(5, 10);

        // collaborative_navigation::System_global_plan global_plan;
        // global_plan.path_points.poses = 

    }

} // namespace DSTTMR
