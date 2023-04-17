#include "path_generator.h"
#include <tf/tf.h>

namespace DSTTMR
{
    PathGenerator& PathGenerator::getInstance()
    {
        static PathGenerator unitInstance;
        return unitInstance;
    }

    bool PathGenerator::prunePatternPath(std::vector<geometry_msgs::PoseStamped>& pattern_points_in_global, const double& robot_pose_x, const double& robot_pose_y, double prune_threshold)
    {
        if (pattern_points_in_global.empty())
        {
            ROS_DEBUG("Pattern points in global are empty. Pruning failed.");
            return false;
        }

        std::vector<geometry_msgs::PoseStamped>::iterator it = pattern_points_in_global.begin();
        std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
        while (it != pattern_points_in_global.end())
        {
            double dx = robot_pose_x - it->pose.position.x;
            double dy = robot_pose_y - it->pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < prune_threshold)
            {
                erase_end = it;
            }
            ++it;
        }
        if (erase_end == pattern_points_in_global.end())
        {
            ROS_INFO("No pattern points have been walked through. Cancel pruning.");
            return false;
        }

        if (erase_end != pattern_points_in_global.begin())
        {
            ROS_INFO("Pattern points pruned.");
            pattern_points_in_global.erase(pattern_points_in_global.begin(), erase_end);
        }

        return true;
    }

    std::vector<geometry_msgs::PoseStamped> PathGenerator::createCirclePath(const double& radius = 1, const int& int_points = 10)
    {

    }

    // std::vector<geometry_msgs::PoseStamped> PathGenerator::createNormalWorkTurnRightPath(const nav_msgs::Path& waypoints, double radius, int num_points)
    // {
    //     // number of received waypoints: 1 or 2
    //     // >>> use first waypoint to design pattern, and connect pattern points to second waypoint.

    //     // Pattern design
    //     double useless_roll, useless_pitch;
    //     std::vector<geometry_msgs::PoseStamped> pattern_points;
    //     for(float i=0; i<=num_points; i++)
    //     {
    //         geometry_msgs::PoseStamped point;
    //         point.header = waypoints.poses.front().header;
    //         point.pose.position.x = radius*cos(M_PI - i/num_points*M_PI/2.0);
    //         point.pose.position.y = radius*sin(M_PI - i/num_points*M_PI/2.0) - radius;
    //         double yaw_inRad = (M_PI/2 - i/num_points*M_PI/2);
    //         point.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_inRad);
    //         pattern_points.push_back(point);
    //     }
    //     double dyaw_inRad;
    //     MathTools::MathTool::transformQuaternionToEuler(waypoints.poses.front().pose.orientation, 
    //                             useless_roll,
    //                             useless_pitch,
    //                             dyaw_inRad);
    //     transformPatternPoints(waypoints.poses.front().pose.position.x,
    //                         waypoints.poses.front().pose.position.y,
    //                         dyaw_inRad,
    //                         pattern_points);
        
    //     if(waypoints.poses.size()>1)
    //     {
    //         for(auto i=1; i<waypoints.poses.size(); i++)
    //         {
    //             pattern_points.emplace_back(waypoints.poses[i]);
    //         }
    //     }

    //     return pattern_points;
    // }

    void PathGenerator::transformPatternPoints(const double& dx, const double& dy, const double& dyaw_inRad, std::vector<geometry_msgs::PoseStamped>& points)
    {
        double c = cos(dyaw_inRad);
        double s = sin(dyaw_inRad);
        for(auto& point : points)
        {
            double new_x = c*point.pose.position.x - s*point.pose.position.y + dx;
            double new_y = s*point.pose.position.x + c*point.pose.position.y + dy;
            point.pose.position.x = new_x;
            point.pose.position.y = new_y;

            double useless_roll, useless_pitch, old_yaw_inRad;
            MathTools::MathTool::transformQuaternionToEuler(point.pose.orientation, 
                                    useless_roll,
                                    useless_pitch,
                                    old_yaw_inRad);
            point.pose.orientation = tf::createQuaternionMsgFromYaw(old_yaw_inRad + dyaw_inRad);
        }
    }
    
    void PathGenerator::interpolatePath(std::vector<geometry_msgs::PoseStamped>& waypoints)
    {
        std::vector<geometry_msgs::PoseStamped> temp_path;
        // notice that stop condition: it+1
        for(auto it = waypoints.begin(); (it+1) != waypoints.end(); ++it)
        {
            double x1 = it->pose.position.x;
            double y1 = it->pose.position.y;
            double x2 = (it+1)->pose.position.x;
            double y2 = (it+1)->pose.position.y;
            double dist =  hypot(x1-x2, y1-y2);
            int num_wpts = dist * points_per_meter_;
            auto p = *it;
            temp_path.push_back(p);
            for (int j = 0; j < num_wpts - 2; j++)
            {
                p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
                p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
                temp_path.push_back(p);
            }
        }
        for (size_t i = 0; i < temp_path.size(); i++)
            temp_path[i].header.seq = static_cast<int>(i);
        temp_path.push_back(waypoints.back());
        waypoints = temp_path;
    }

} // namespace DSTTMR
