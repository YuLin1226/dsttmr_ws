#if !defined(_VISUALIZATION_H_)
#define _VISUALIZATION_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace DSTTMR
{
    class PathVisualization
    {
        private:
            ros::Publisher visualize_path_publisher_; 
            ros::NodeHandle private_nh_;

        public:
            PathVisualization(ros::NodeHandle& private_nh);

            void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path);
            void createAndPublishArrowMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path);

    };
} // namespace DSTTMR



#endif // _VISUALIZATION_H_
