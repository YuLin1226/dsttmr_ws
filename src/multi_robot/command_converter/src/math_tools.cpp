#include "command_converter/math_tools.h"


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


} // namespace CalculationTools
