#include "../include/command_converter/convert_velocity_command.h"
#include <cmath>


namespace ConvertVelocityCommand
{
    DualSteeringToDiffDrive::DualSteeringToDiffDrive():
    initialized_(false),
    private_nh_("~"),
    callbackQueue_(true),
    asyncSpinner_(NUM_THREADS, &callbackQueue_)
    {
        if(!initialized_)
        {
            initialization();
        }
    }

    DualSteeringToDiffDrive::~DualSteeringToDiffDrive()
    {
    }

    void DualSteeringToDiffDrive::initialization()
    {
        if(!getRobotParameters())
        {
            // Throw error.
            return;
        }
        initialized_ = true;
    }

    bool DualSteeringToDiffDrive::getRobotParameters()
    {
        // retrive parameters.
        return true;
    }

    void DualSteeringToDiffDrive::convertSteerLinearVelocityToDiffDriveCommand(double& linear_velocity_command, double& steer_angle_command, double& current_steer_pos, double& dt, double& vx, double&w)
    {
        vx = linear_velocity_command;
        w = (steer_angle_command - current_steer_pos)/dt;
    }

    void DualSteeringToDiffDrive::getDualWheelsCommand(double& linear_velocity_x_command, double& linear_velocity_y_command, double& angular_velocity_z_command)
    {
        wheels_velocity_command_.front_wheel_linear_velocity_x = linear_velocity_x_command;
        wheels_velocity_command_.front_wheel_linear_velocity_y = linear_velocity_y_command + angular_velocity_z_command * robot_distance_ / 2;
        wheels_velocity_command_.rear_wheel_linear_velocity_x = linear_velocity_x_command;
        wheels_velocity_command_.rear_wheel_linear_velocity_y =  linear_velocity_y_command - angular_velocity_z_command * robot_distance_ / 2;
    }

    void DualSteeringToDiffDrive::convertWheelLinearVelocityToSteerLinearVelocityCommand(double& wheel_linear_velocity_x_command, double& wheel_linear_velocity_y_command, double& linear_velocity_command, double& steer_angle_command)
    {
        linear_velocity_command = std::sqrt(wheel_linear_velocity_x_command*wheel_linear_velocity_x_command + wheel_linear_velocity_y_command*wheel_linear_velocity_y_command);
        steer_angle_command = std::atan2(wheel_linear_velocity_y_command, wheel_linear_velocity_x_command);
    }


    void DualSteeringToDiffDrive::updateRobotDistanceByProbabilistics(geometry_msgs::PoseWithCovarianceStamped& coworker, geometry_msgs::PoseWithCovarianceStamped& myself)
    {
        // argument should be changed to PoseTwistWithCovarianceStamped type.

        // How to calculate new robot distance by probabilistic method?
    }

    void DualSteeringToDiffDrive::combineTwoProbabilisticDistributions(const Eigen::VectorXd& mu1, const Eigen::MatrixXd& cov1, const Eigen::VectorXd& mu2, const Eigen::MatrixXd& cov2, const Eigen::VectorXd& combined_mu, const Eigen::MatrixXd& combined_cov)
    {
        // Method noted in my hackmd: https://hackmd.io/qft09FZtSSyRbofPZj7YbA
    }

} // namespace ConvertVelocityCommand
