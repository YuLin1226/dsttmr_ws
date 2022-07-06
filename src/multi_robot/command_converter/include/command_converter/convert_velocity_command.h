#ifndef _CONVERT_VELOCITY_COMMAND_H_
#define _CONVERT_VELOCITY_COMMAND_H_

#include "ros/ros.h"
#include <ros/callback_queue.h>


namespace ConvertVelocityCommand
{


class DualSteeringToDiffDrive
{
    private:
        // Number of threads for this class
        static const int NUM_THREADS = 1;

        // Ros callback queue for this class
        ros::CallbackQueue callbackQueue_;

        // Ros async spinner
        ros::AsyncSpinner asyncSpinner_;

        // Ros private node handle
        ros::NodeHandle private_nh_;

        // Initialization state.
        bool initialized_;

        // Robot distance.
        double robot_distance_;

        // dual wheel command struct
        struct wheels_velocity
        {
            double front_wheel_linear_velocity_x;
            double front_wheel_linear_velocity_y;
            double rear_wheel_linear_velocity_x;
            double rear_wheel_linear_velocity_y;

            wheels_velocity():
            front_wheel_linear_velocity_x(0.0),
            front_wheel_linear_velocity_y(0.0),
            rear_wheel_linear_velocity_x(0.0),
            rear_wheel_linear_velocity_y(0.0)
            {
            }
        } wheels_velocity_command_;

        // wheels_velocity_storage
        

    public:
        /**
         * @brief Constructor.
         */
        DualSteeringToDiffDrive();

        /**
         * @brief Destructor.
         */
        ~DualSteeringToDiffDrive();

        /**
         * @brief Initialization function.
         */
        void initialization();

        /**
         * @brief Retrive robots' parameters from ros param server.
         */
        bool getRobotParameters();

        /**
         * @brief convert command from dual steer kinematics to diff-drive kinematics.
         * @param linear_velocity_x x axis linear velocity wrt to global frame
         * @param linear_velocity_y y axis linear velocity wrt to global frame
         * @param angular_velocity_z angular velocity
         */
        void convertCommand(double& linear_velocity_x, double& linear_velocity_y, double& angular_velocity_z);

        /**
         * @brief dual steering wheel kinematics for velocity command.
         * @param linear_velocity_x x axis linear velocity
         * @param linear_velocity_y y axis linear velocity
         * @param angular_velocity_z angular velocity
         */
        void getDualWheelsCommand(double& linear_velocity_x, double& linear_velocity_y, double& angular_velocity_z);
        

};
} // namespace ConvertVelocityCommand
#endif