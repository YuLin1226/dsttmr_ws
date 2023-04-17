#if !defined(_PATH_GENERATOR_H_)
#define _PATH_GENERATOR_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace DSTTMR
{
    class PathGenerator
    {
        private:
            unsigned int points_per_meter_ = 10; // default: 10
        public:
            static PathGenerator& getInstance();            
            PathGenerator(PathGenerator const&) =delete;
            void operator=(PathGenerator const&) =delete;
            bool prunePatternPath(std::vector<geometry_msgs::PoseStamped>& pattern_points_in_global, const double& robot_pose_x, const double& robot_pose_y, double prune_threshold=0.03);
            // customized pattern path
            std::vector<geometry_msgs::PoseStamped> createCirclePath(const double& radius = 1, const int& int_points = 10);
        private:
            PathGenerator() {};
            void transformPatternPoints(const double& dx, const double& dy, const double& dyaw_inRad, std::vector<geometry_msgs::PoseStamped>& points);
            void interpolatePath(std::vector<geometry_msgs::PoseStamped>& waypoints);
    };
} // namespace DSTTMR


#endif // _PATH_GENERATOR_H_
