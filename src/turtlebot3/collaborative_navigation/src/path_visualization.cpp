#include "path_visualization.h"
#include <visualization_msgs/MarkerArray.h>

namespace DSTTMR
{
    PathVisualization::PathVisualization(ros::NodeHandle& private_nh)
    : private_nh_(private_nh)
    {
        visualize_path_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    }

    void PathVisualization::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
    {
        // clear previous markers
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;
        marker.header = path[0].header;
        marker.ns = "/move_base/waypoint_global_planner";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.id = 0;
        markers.markers.push_back(marker);
        visualize_path_publisher_.publish(markers);
        marker.action = visualization_msgs::Marker::ADD;
        markers.markers.clear();

        for (size_t i = 0; i < path.size(); i++)
        {
            marker.id = i;
            marker.pose.position = path[i].pose.position;
            markers.markers.push_back(marker);
        }

        visualize_path_publisher_.publish(markers);
    }

    void PathVisualization::createAndPublishArrowMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
    {
        // clear previous markers
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;
        marker.header = path[0].header;
        marker.ns = "/move_base/waypoint_global_planner";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker.scale.x = 1.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.id = 0;
        markers.markers.push_back(marker);
        visualize_path_publisher_.publish(markers);
        marker.action = visualization_msgs::Marker::ADD;
        markers.markers.clear();

        for (size_t i = 0; i < path.size(); i++)
        {
            if(i == path.size()-1)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            marker.id = i;
            marker.pose = path[i].pose;
            markers.markers.push_back(marker);
        }

        visualize_path_publisher_.publish(markers);
    }


} // namespace DSTTMR
