#ifndef _MATH_TOOLS_H_
#define _MATH_TOOLS_H_

#include <Eigen/Dense>


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




};
} // namespace CalculationTools
#endif