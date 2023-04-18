#include "system_global_planner.h"
#include <nav_msgs/Path.h>

namespace DSTTMR
{
    SystemGlobalPlanner::SystemGlobalPlanner(ros::NodeHandle& private_nh)
    : private_nh_(private_nh)
    {
        agents_global_planner_pub_ = private_nh_.advertise<nav_msgs::Path>("/global_plan", 1);
        // tf
        private_nh_.param<std::string>("/first_robot_frame_name", first_robot_frame_name_, "/first_robot_base_footprint");
        private_nh_.param<std::string>("/second_robot_frame_name", second_robot_frame_name_, "/second_robot_base_footprint");
        first_robot_tf_listener_ = std::make_shared<TF_LISTENER>("/map", first_robot_frame_name_);
        second_robot_tf_listener_ = std::make_shared<TF_LISTENER>("/map", second_robot_frame_name_);
    }

    void SystemGlobalPlanner::publishGlobalPlan()
    {
        auto& path_generator = DSTTMR::PathGenerator::getInstance();
        // step 1: create template path.
        auto path_points = path_generator.createCirclePath(5, 10);
        // step 2: move & rotate path
        double init_x, init_y, init_yaw;
        private_nh_.param<double>("/system_global_plan_first_position_x", init_x, 0.0);
        private_nh_.param<double>("/system_global_plan_first_position_y", init_y, 0.0);
        private_nh_.param<double>("/system_global_plan_first_rotation_yaw", init_yaw, 0.0);
        path_generator.transformPatternPoints(init_x, init_y, init_yaw, path_points);
        // step 3: interpolation
        path_generator.interpolatePath(path_points);
        // step 4: prune
        RobotPose first_robot_pose, second_robot_pose;
        first_robot_tf_listener_->updateRobotPose(first_robot_pose.position_x, first_robot_pose.position_y, first_robot_pose.rotation_yaw);
        second_robot_tf_listener_->updateRobotPose(second_robot_pose.position_x, second_robot_pose.position_y, second_robot_pose.rotation_yaw);
        robot_pose_.position_x = (first_robot_pose.position_x + second_robot_pose.position_x)/2;
        robot_pose_.position_y = (first_robot_pose.position_y + second_robot_pose.position_y)/2;
        path_generator.prunePatternPath(path_points, robot_pose_.position_x, robot_pose_.position_y);
        // step 5: publish
        nav_msgs::Path global_plan;
        // header?
        global_plan.poses = path_points;
        agents_global_planner_pub_.publish(global_plan);
    }

} // namespace DSTTMR

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_global_planner_node");
    ros::NodeHandle private_nh("~");
    auto system_global_planner = std::make_shared<DSTTMR::SystemGlobalPlanner>(private_nh);
    // publish data
    system_global_planner->publishGlobalPlan();
    return 0;
};
