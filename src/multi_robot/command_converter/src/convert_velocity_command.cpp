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

    void DualSteeringToDiffDrive::convertCommand(double& linear_velocity_x, double& linear_velocity_y, double& angular_velocity_z)
    {
        if(fabs(angular_velocity_z) > 0)
        {
            getDualWheelsCommand(linear_velocity_x, linear_velocity_y, angular_velocity_z);
            double linear_velocity_sum = std::sqrt(linear_velocity_x*linear_velocity_x + linear_velocity_y*linear_velocity_y);
            double linear_velocity_theta = std::atan2(linear_velocity_y, linear_velocity_x);
            double velocity_radius = linear_velocity_sum/angular_velocity_z;

            double front_velocity_radius = std::sqrt((robot_distance_/2)*(robot_distance_/2) 
                                            +velocity_radius*velocity_radius 
                                            -2*(robot_distance_/2)*velocity_radius*cos(linear_velocity_theta + M_PI/2));
            
            double rear_velocity_radius = std::sqrt((robot_distance_/2)*(robot_distance_/2) 
                                           +velocity_radius*velocity_radius 
                                           -2*(robot_distance_/2)*velocity_radius*cos(linear_velocity_theta - M_PI/2));

            double front_robot_linear_velocity = std::sqrt(wheels_velocity_command_.front_wheel_linear_velocity_x*wheels_velocity_command_.front_wheel_linear_velocity_x
                                                          +wheels_velocity_command_.front_wheel_linear_velocity_y*wheels_velocity_command_.front_wheel_linear_velocity_y); 

            double front_robot_angular_velocity = front_robot_angular_velocity / front_velocity_radius;

            double rear_robot_angular_velocity = rear_robot_angular_velocity / rear_velocity_radius;

            double rear_robot_linear_velocity = std::sqrt(wheels_velocity_command_.rear_wheel_linear_velocity_x*wheels_velocity_command_.rear_wheel_linear_velocity_x
                                                         +wheels_velocity_command_.rear_wheel_linear_velocity_y*wheels_velocity_command_.rear_wheel_linear_velocity_y);
        }
        else
        {
            // No idea ...
        }
    }

    void DualSteeringToDiffDrive::getDualWheelsCommand(double& linear_velocity_x, double& linear_velocity_y, double& angular_velocity_z)
    {
        wheels_velocity_command_.front_wheel_linear_velocity_x = linear_velocity_x;
        wheels_velocity_command_.front_wheel_linear_velocity_y = linear_velocity_y + angular_velocity_z * robot_distance_ / 2;
        wheels_velocity_command_.rear_wheel_linear_velocity_x = linear_velocity_x;
        wheels_velocity_command_.rear_wheel_linear_velocity_y =  linear_velocity_y - angular_velocity_z * robot_distance_ / 2;
    }


} // namespace ConvertVelocityCommand
