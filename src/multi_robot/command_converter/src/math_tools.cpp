#include "command_converter/math_tools.h"
#include <tf/tf.h>

namespace CalculationTools
{
    MathTools::MathTools(/* args */)
    {
    }

    MathTools::~MathTools()
    {
    }

    void MathTools::combineTwoProbabilisticDistributions(const Eigen::VectorXd& mu1, const Eigen::MatrixXd& cov1, const Eigen::VectorXd& mu2, const Eigen::MatrixXd& cov2, Eigen::VectorXd& combined_mu, Eigen::MatrixXd& combined_cov)
    {
        combined_cov = (cov1.inverse() + cov2.inverse()).inverse();
        Eigen::VectorXd w1 = cov2*(cov1 + cov2).inverse();
        Eigen::VectorXd w2 = cov1*(cov1 + cov2).inverse();
        combined_mu = w1*mu1 + w2*mu2;
    }

    void MathTools::transformQuaternionToEuler(geometry_msgs::Quaternion& orientation, double& roll, double& pitch, double& yaw)
    {
        tf::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

    void MathTools::organizePoseCovToEigenData(geometry_msgs::PoseWithCovariance& PoseCov, Eigen::VectorXd& mu, Eigen::MatrixXd& cov)
    {
        mu(0) = PoseCov.pose.position.x;
        mu(1) = PoseCov.pose.position.y;
        mu(2) = PoseCov.pose.position.z;
        transformQuaternionToEuler(PoseCov.pose.orientation, mu(3), mu(4), mu(5));
        cov << 
        PoseCov.covariance[0], PoseCov.covariance[1], PoseCov.covariance[2], PoseCov.covariance[3], PoseCov.covariance[4], PoseCov.covariance[5],
        PoseCov.covariance[6], PoseCov.covariance[7], PoseCov.covariance[8], PoseCov.covariance[9], PoseCov.covariance[10], PoseCov.covariance[11],
        PoseCov.covariance[12], PoseCov.covariance[13], PoseCov.covariance[14], PoseCov.covariance[15], PoseCov.covariance[16], PoseCov.covariance[17],
        PoseCov.covariance[18], PoseCov.covariance[19], PoseCov.covariance[20], PoseCov.covariance[21], PoseCov.covariance[22], PoseCov.covariance[23],
        PoseCov.covariance[24], PoseCov.covariance[25], PoseCov.covariance[26], PoseCov.covariance[27], PoseCov.covariance[28], PoseCov.covariance[29],
        PoseCov.covariance[30], PoseCov.covariance[31], PoseCov.covariance[32], PoseCov.covariance[33], PoseCov.covariance[34], PoseCov.covariance[35];
    }


} // namespace CalculationTools
