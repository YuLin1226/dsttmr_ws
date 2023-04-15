#if !defined(_AGENT_GLOBAL_PLANNER_H_)
#define _AGENT_GLOBAL_PLANNER_H_


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_layer.h>
#include "waypoint_global_planner/Waypoint_msg.h"


namespace DSTTMR
{

    class AgentGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        AgentGlobalPlanner();
        AgentGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        ~AgentGlobalPlanner() {}
        
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start_pose,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

    private:
        void waypointWithOrientationCallback(const geometry_msgs::PoseStamped::ConstPtr &waypoint);
        void interpolatePath(nav_msgs::Path &path);

    private:
        static const int NUM_THREADS = 1;
        ros::AsyncSpinner asyncSpinner_;
        ros::CallbackQueue callbackQueue_;
        bool initialized_;
        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;

        base_local_planner::WorldModel *world_model_;
        boost::shared_ptr<costmap_2d::CostmapLayer> static_layer_;
        ros::NodeHandle private_nh_;
        ros::Subscriber waypoint_with_orientation_sub_;
        ros::Publisher receive_waypoint_pub_;
        ros::Publisher goal_pub_;
        ros::Publisher plan_pub_;
        double epsilon_;
        int waypoints_per_meter_;
        std::vector<geometry_msgs::PoseStamped> waypoints_;
        nav_msgs::Path path_;
        bool clear_waypoints_;

    };

} // namespace DSTTMR

#endif // _AGENT_GLOBAL_PLANNER_H_
