#if !defined(_AGENT_GLOBAL_PLANNER_H_)
#define _AGENT_GLOBAL_PLANNER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <string>
#include <memory>
#include "tf_listener.h"


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
        void systemGlobalPlanCallback(const nav_msgs::Path::ConstPtr& plan);
        void interpolatePath(nav_msgs::Path &path);
        void computeAgentGlobalPlan(nav_msgs::Path &path);
        double getRobotDistance();

    private:
        // async setting
        static const int NUM_THREADS = 1;
        ros::AsyncSpinner asyncSpinner_;
        ros::CallbackQueue callbackQueue_;
        // ros setting
        ros::NodeHandle private_nh_;
        ros::Subscriber system_global_plan_sub_;
        ros::Publisher goal_pub_;
        ros::Publisher plan_pub_;
        // others
        bool initialized_;
        double epsilon_;
        int waypoints_per_meter_;
        std::vector<geometry_msgs::PoseStamped> waypoints_;
        nav_msgs::Path path_;
        bool clear_waypoints_;

        std::string own_frame_name_;
        std::string partner_frame_name_;
        std::shared_ptr<TF_LISTENER> own_tf_listener_;
        std::shared_ptr<TF_LISTENER> partner_tf_listener_;


    };

} // namespace DSTTMR

#endif // _AGENT_GLOBAL_PLANNER_H_
