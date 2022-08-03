#ifndef _MATH_TOOLS_H_
#define _MATH_TOOLS_H_

#include <Eigen/Dense>
#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>


namespace CalculationTools
{

class MathTools
{
private:
    



public:
    MathTools(/* args */);
    ~MathTools();



    /**
     * @brief Combine  distributions by baysian method.
     * @param mu mean of distribution.
     * @param cov covariance matrix of the distribution.
     */
    void combineTwoProbabilisticDistributions(const Eigen::VectorXd& mu1, const Eigen::MatrixXd& cov1, const Eigen::VectorXd& mu2, const Eigen::MatrixXd& cov2, Eigen::VectorXd& combined_mu, Eigen::MatrixXd& combined_cov);

    /**
     * @brief Transform quaternion to euler angles.
     * @param orientation quaternion input
     * @param roll euler angle
     * @param pitch euler angle
     * @param yaw euler angle
     */
    void transformQuaternionToEuler(geometry_msgs::Quaternion& orientation, double& roll, double& pitch, double& yaw);

    void organizePoseCovToEigenData(geometry_msgs::PoseWithCovariance& PoseCov, Eigen::VectorXd& mu, Eigen::MatrixXd& cov);
};
} // namespace CalculationTools
#endif