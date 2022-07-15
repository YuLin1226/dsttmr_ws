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

    void DualSteeringToDiffDrive::convertLinearVelocityToDiffDriveCommand(double& linear_velocity_command, double& steer_angle_command, double& dt)
    {
        // linear_velocity remains
        // double angular_velocity = (steer_angle_command - current_steer_angle)/dt;
    }

    void DualSteeringToDiffDrive::getDualWheelsCommand(double& linear_velocity_x_command, double& linear_velocity_y_command, double& angular_velocity_z_command)
    {
        wheels_velocity_command_.front_wheel_linear_velocity_x = linear_velocity_x_command;
        wheels_velocity_command_.front_wheel_linear_velocity_y = linear_velocity_y_command + angular_velocity_z_command * robot_distance_ / 2;
        wheels_velocity_command_.rear_wheel_linear_velocity_x = linear_velocity_x_command;
        wheels_velocity_command_.rear_wheel_linear_velocity_y =  linear_velocity_y_command - angular_velocity_z_command * robot_distance_ / 2;
    }

    void DualSteeringToDiffDrive::convertWheelLinearVelocityToSteerLinearVelocityCommand(double& wheel_linear_velocity_x_command, double& wheel_linear_velocity_y_command)
    {
        double linear_velocity = std::sqrt(wheel_linear_velocity_x_command*wheel_linear_velocity_x_command + wheel_linear_velocity_y_command*wheel_linear_velocity_y_command);
        double steer_angle = std::atan2(wheel_linear_velocity_y_command, wheel_linear_velocity_x_command);
    }


} // namespace ConvertVelocityCommand
