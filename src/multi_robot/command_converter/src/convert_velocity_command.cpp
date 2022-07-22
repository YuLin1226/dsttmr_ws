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

        // Information:
        // pose & twist from coworker (time delay, large covariance).
        // pose & twist from self (current time, small covariance).
        // These information are independent, so I think a system model is needed for processing robot distance by methods similar to kalman filter.

        // Others:
        // If informations are robot_distances calculated from different agent, these are dependent, so they can be processed by covariance intersection or similar methods.
        
    }


} // namespace ConvertVelocityCommand
