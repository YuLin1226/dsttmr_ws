#include "../include/command_converter/convert_velocity_command.h"


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

    void DualSteeringToDiffDrive::convertCommand(double& linear_velocity_x, double& linear_velocity_y, double& angular_velocity_z)
    {
        getDualWheelsCommand(linear_velocity_x, linear_velocity_y, angular_velocity_z);
        // double linear_velocity_sum = std::sqrt(linear_velocity_x*linear_velocity_x + linear_velocity_y*linear_velocity_y);
        // double linear_velocity_theta = std::atan2(linear_velocity_y, linear_velocity_x);
        // double velocity_radius = linear_velocity_sum/angular_velocity_z;
    }

    void DualSteeringToDiffDrive::getDualWheelsCommand(double& linear_velocity_x, double& linear_velocity_y, double& angular_velocity_z)
    {
        wheels_velocity_command_.front_wheel_linear_velocity_x = linear_velocity_x;
        wheels_velocity_command_.front_wheel_linear_velocity_y = linear_velocity_y + angular_velocity_z * robot_distance_ / 2;
        wheels_velocity_command_.rear_wheel_linear_velocity_x = linear_velocity_x;
        wheels_velocity_command_.rear_wheel_linear_velocity_y =  linear_velocity_y - angular_velocity_z * robot_distance_ / 2;
    }


} // namespace ConvertVelocityCommand
