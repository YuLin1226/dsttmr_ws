#include "agent_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(DSTTMR::AgentGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace DSTTMR
{
    
AgentGlobalPlanner::AgentGlobalPlanner()
: costmap_ros_(NULL)
, initialized_(false)
, clear_waypoints_(false)
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

void AgentGlobalPlanner::systemGlobalPlanCallback(const nav_msgs::Path::ConstPtr& system_path)
{
    if (clear_waypoints_)
    {
        waypoints_.clear();
        clear_waypoints_ = false;
    }

    if (waypoints_.size() > 0)
    {
        geometry_msgs::Pose *last_point = &(waypoints_.end()-1)->pose;
        // calculate distance between latest two waypoints and check if it surpasses the threshold epsilon
        double dist = hypot(waypoint->pose.position.x - last_point->position.x, waypoint->pose.position.y - last_point->position.y);
        if (dist < epsilon_)
        {
            path_.header = waypoint->header;
            path_.poses.clear();
            path_.poses.insert(path_.poses.end(), waypoints_.begin(), waypoints_.end());
            goal_pub_.publish(waypoints_.back());
            clear_waypoints_ = true;
            ROS_INFO("Published waypoint planner goal pose");
            // create and publish markers
            visualization_->createAndPublishArrowMarkersFromPath(waypoints_);
            return;
        }
    }

    // add waypoint to the waypoint vector
    waypoints_.push_back(geometry_msgs::PoseStamped());
    waypoints_.back().header = waypoint->header;
    waypoints_.back().pose.position = waypoint->pose.position;
    waypoints_.back().pose.orientation = waypoint->pose.orientation;

    // create and publish markers
    visualization_->createAndPublishArrowMarkersFromPath(waypoints_);
}

void AgentGlobalPlanner::computeAgentGlobalPlan(nav_msgs::Path &system_path)
{
    auto robot_distance = getRobotDistance();
    nav_msgs::Path front_agent_path, rear_agent_path;
    computeAgentsGlobalPlan(front_agent_path, rear_agent_path, system_path, robot_distance);
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

void AgentGlobalPlanner::computeAgentsGlobalPlan(nav_msgs::Path &front_agent_path, nav_msgs::Path &rear_agent_path, nav_msgs::Path &system_path, double& robot_distance)
{
    front_agent_path.header = system_path.header;
    rear_agent_path.header = system_path.header;
    for(auto& pose : system_path.poses)
    {
        // calculate front robot pose
        // calculate rear robot pose
        // about orientation, we can follow the original one.
    }
}

void AgentGlobalPlanner::selectAgentGlobalPlan(nav_msgs::Path &front_agent_path, nav_msgs::Path &rear_agent_path)
{
    double own_position_x, own_position_y, own_rotation_yaw;
    own_tf_listener_->updateRobotPose(own_position_x, own_position_y, own_rotation_yaw);
    // design a if-else logic to select a proper agent_path.
}

} // namespace DSTTMR




