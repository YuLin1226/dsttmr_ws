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
        math_tools_ = std::make_shared<CalculationTools::MathTools>();
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


    void DualSteeringToDiffDrive::updateRobotDistanceByProbabilistics(geometry_msgs::PoseWithCovariance& coworker_posecov_from_coworker, 
                                                                      geometry_msgs::PoseWithCovariance& myself_posecov_from_coworker,
                                                                      geometry_msgs::PoseWithCovariance& coworker_posecov_from_myself, 
                                                                      geometry_msgs::PoseWithCovariance& myself_posecov_from_myself)
    {
        /* Calculation Method:
        1. Get coworker pose with cov and twist with cov to compute current pose of coworker.   
        (This may involve time delay, so twist msg needs to be used to update the pose.)
        2. Get myself pose with cov from coworker's observation.
        3. Observe coworker pose with cov.
        4. Get myself pose with cove from localization.

        Use 1 & 3 to get filtered pose with cov of coworker.
            2 & 4 to get filtered pose with cov of myself.

        Use 2 filtered poses to calculate robot distance and update it.
        */
        Eigen::VectorXd coworker_pose_from_coworker_mu = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd coworker_pose_from_coworker_cov = Eigen::MatrixXd::Zero(6,6);
        Eigen::VectorXd myself_pose_from_coworker_mu = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd myself_pose_from_coworker_cov = Eigen::MatrixXd::Zero(6,6);
        Eigen::VectorXd coworker_pose_from_myself_mu = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd coworker_pose_from_myself_cov = Eigen::MatrixXd::Zero(6,6);
        Eigen::VectorXd myself_pose_from_myself_mu = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd myself_pose_from_myself_cov = Eigen::MatrixXd::Zero(6,6);

        math_tools_->organizePoseCovToEigenData(coworker_posecov_from_coworker, coworker_pose_from_coworker_mu, coworker_pose_from_coworker_cov);
        math_tools_->organizePoseCovToEigenData(myself_posecov_from_coworker, myself_pose_from_coworker_mu, myself_pose_from_coworker_cov);
        math_tools_->organizePoseCovToEigenData(coworker_posecov_from_myself, coworker_pose_from_myself_mu, coworker_pose_from_myself_cov);
        math_tools_->organizePoseCovToEigenData(myself_posecov_from_myself, myself_pose_from_myself_mu, myself_pose_from_myself_cov);
        
        Eigen::VectorXd coworker_pose_filtered_mu = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd coworker_pose_filtered_cov = Eigen::MatrixXd::Zero(6,6);
        Eigen::VectorXd myself_pose_filtered_mu = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd myself_pose_filtered_cov = Eigen::MatrixXd::Zero(6,6);

        math_tools_->combineTwoProbabilisticDistributions(coworker_pose_from_coworker_mu, coworker_pose_from_coworker_cov, coworker_pose_from_myself_mu, coworker_pose_from_myself_cov, coworker_pose_filtered_mu, coworker_pose_filtered_cov);
        math_tools_->combineTwoProbabilisticDistributions(myself_pose_from_coworker_mu, myself_pose_from_coworker_cov, myself_pose_from_myself_mu, myself_pose_from_myself_cov, myself_pose_filtered_mu, myself_pose_filtered_cov);
        
        robot_distance_ = std::sqrt((coworker_pose_filtered_mu[0] - myself_pose_filtered_mu[0])*(coworker_pose_filtered_mu[0] - myself_pose_filtered_mu[0]) + (coworker_pose_filtered_mu[1] - myself_pose_filtered_mu[1])*(coworker_pose_filtered_mu[1] - myself_pose_filtered_mu[1]));
    }

} // namespace ConvertVelocityCommand
