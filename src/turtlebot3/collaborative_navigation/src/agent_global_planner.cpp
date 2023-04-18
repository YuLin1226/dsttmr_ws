#include "agent_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

PLUGINLIB_EXPORT_CLASS(DSTTMR::AgentGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace DSTTMR
{
    
AgentGlobalPlanner::AgentGlobalPlanner()
: initialized_(false)
, private_nh_("~")
, callbackQueue_(true)
, asyncSpinner_(NUM_THREADS, &callbackQueue_)
{
}

AgentGlobalPlanner::AgentGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
: private_nh_("~"+name)
, callbackQueue_(true)
, asyncSpinner_(NUM_THREADS, &callbackQueue_)
{
    initialize(name, costmap_ros);
}

void AgentGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        // set async
        private_nh_.setCallbackQueue(&callbackQueue_);
        asyncSpinner_.start();
    
        // tf
        private_nh_.param("/own_frame_name", own_frame_name_, "/first_robot_base_footprint");
        private_nh_.param("/partner_frame_name", partner_frame_name_, "/second_robot_base_footprint");
        own_tf_listener_ = std::make_shared<TF_LISTENER>("/map", own_frame_name_);
        partner_tf_listener_ = std::make_shared<TF_LISTENER>("/map", partner_frame_name_);
    
        // pub & sub
        ros::NodeHandle nh;
        system_global_plan_sub_ = private_nh_.subscribe("/global_way_points", 100, &AgentGlobalPlanner::systemGlobalPlanCallback, this);
        goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        plan_pub_ = private_nh_.advertise<nav_msgs::Path>("global_plan", 1);
    
        // ptr
        visualization_ = std::make_shared<DSTTMR::PathVisualization>(private_nh_);

        ROS_INFO("Waypoint planner has been initialized");
        initialized_ = true;
    }
    else
    {
        ROS_WARN("This waypoint planner has already been initialized");
    }
}

bool AgentGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    path_.poses.insert(path_.poses.begin(), start_pose);
    interpolatePath(path_);
    plan_pub_.publish(path_);
    plan = path_.poses;
    ROS_INFO("Published waypoints global plan");
    return true;
}

void AgentGlobalPlanner::interpolatePath(nav_msgs::Path& path)
{
    std::vector<geometry_msgs::PoseStamped> temp_path;
    for (int i = 0; i < static_cast<int>(path.poses.size()-1); i++)
    {
        // calculate distance between two consecutive waypoints
        double x1 = path.poses[i].pose.position.x;
        double y1 = path.poses[i].pose.position.y;
        double x2 = path.poses[i+1].pose.position.x;
        double y2 = path.poses[i+1].pose.position.y;
        double dist =  hypot(x1-x2, y1-y2);
        int num_wpts = dist * waypoints_per_meter_;

        temp_path.push_back(path.poses[i]);
        // set the orientation of inserted points between pose [i & i+1] to be same as pose[i].
        geometry_msgs::PoseStamped p = path.poses[i];
        for (int j = 0; j < num_wpts - 2; j++)
        {
            p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
            p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
            temp_path.push_back(p);
        }
    }

    // update sequence of poses
    for (size_t i = 0; i < temp_path.size(); i++)
        temp_path[i].header.seq = static_cast<int>(i);

    temp_path.push_back(path.poses.back());
    path.poses = temp_path;
}

void AgentGlobalPlanner::systemGlobalPlanCallback(const nav_msgs::Path::ConstPtr& system_path_ptr)
{
    auto system_path = *system_path_ptr;
    auto global_plan = computeAgentGlobalPlan(system_path);

    path_.header = global_plan.header;
    path_.poses.clear();
    path_.poses.insert(path_.poses.end(), global_plan.poses.begin(), global_plan.poses.end());
    goal_pub_.publish(global_plan.poses.back());
    ROS_INFO("Has published the agent_global_plan.");
    // create and publish markers
    visualization_->createAndPublishArrowMarkersFromPath(global_plan.poses);
}

nav_msgs::Path AgentGlobalPlanner::computeAgentGlobalPlan(nav_msgs::Path &system_path)
{
    auto robot_distance = getRobotDistance();
    nav_msgs::Path front_agent_path, rear_agent_path;
    computeRobotModelGlobalPlan(front_agent_path, rear_agent_path, system_path, robot_distance);
    auto& own_global_plan = getOwnGlobalPlan(front_agent_path, rear_agent_path);
    return own_global_plan;
}

double AgentGlobalPlanner::getRobotDistance()
{
    double dist(0.0);
    double own_position_x, own_position_y, own_rotation_yaw;
    double partner_position_x, partner_position_y, partner_rotation_yaw;
    own_tf_listener_->updateRobotPose(own_position_x, own_position_y, own_rotation_yaw);
    partner_tf_listener_->updateRobotPose(partner_position_x, partner_position_y, partner_rotation_yaw);
    double dx = own_position_x - partner_position_x;
    double dy = own_position_y - partner_position_y;
    dist = sqrt(dx*dx + dy*dy);
    return dist;
}

void AgentGlobalPlanner::computeRobotModelGlobalPlan(nav_msgs::Path &front_agent_path, nav_msgs::Path &rear_agent_path, nav_msgs::Path &system_path, double& robot_distance)
{
    front_agent_path.header = system_path.header;
    rear_agent_path.header = system_path.header;
    for(auto& pose : system_path.poses)
    {
        double pose_rotation_yaw;
        pose_rotation_yaw = tf::getYaw(pose.pose.orientation);
        // calculate front robot pose
        geometry_msgs::PoseStamped front_agent_path_pose;
        front_agent_path_pose.header = pose.header;
        front_agent_path_pose.pose.orientation = pose.pose.orientation;
        front_agent_path_pose.pose.position.x = pose.pose.position.x + robot_distance*cos(pose_rotation_yaw);
        front_agent_path_pose.pose.position.y = pose.pose.position.y + robot_distance*sin(pose_rotation_yaw);
        front_agent_path_pose.pose.position.z = pose.pose.position.z;
        front_agent_path.poses.emplace_back(front_agent_path_pose);
        // calculate rear robot pose
        geometry_msgs::PoseStamped rear_agent_path_pose;
        rear_agent_path_pose.header = pose.header;
        rear_agent_path_pose.pose.orientation = pose.pose.orientation;
        rear_agent_path_pose.pose.position.x = pose.pose.position.x + robot_distance*cos(pose_rotation_yaw + M_PI);
        rear_agent_path_pose.pose.position.y = pose.pose.position.y + robot_distance*cos(pose_rotation_yaw + M_PI);
        rear_agent_path_pose.pose.position.z = pose.pose.position.z;
        rear_agent_path.poses.emplace_back(rear_agent_path_pose);
    }
}

nav_msgs::Path& AgentGlobalPlanner::getOwnGlobalPlan(nav_msgs::Path &front_agent_path, nav_msgs::Path &rear_agent_path)
{
    double own_position_x, own_position_y, own_rotation_yaw;
    double partner_position_x, partner_position_y, partner_rotation_yaw;
    own_tf_listener_->updateRobotPose(own_position_x, own_position_y, own_rotation_yaw);
    partner_tf_listener_->updateRobotPose(partner_position_x, partner_position_y, partner_rotation_yaw);
    // if((front - rear).dot(self - partner) > 0) self is front.
    Eigen::Vector2d path_vector(
        front_agent_path.poses.front().pose.position.x - rear_agent_path.poses.front().pose.position.x,
        front_agent_path.poses.front().pose.position.y - rear_agent_path.poses.front().pose.position.y);
    Eigen::Vector2d robot_vector(
        own_position_x - partner_position_x,
        own_position_y - partner_position_y);
    if(path_vector.dot(robot_vector) > 0)
    {
        return front_agent_path;
    }
    else
    {
        return rear_agent_path;
    }
}

} // namespace DSTTMR




