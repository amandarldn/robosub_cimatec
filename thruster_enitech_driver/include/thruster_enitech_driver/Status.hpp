#ifndef THRUSTER_ENITECH_STATUS_HPP
#define THRUSTER_ENITECH_STATUS_HPP

#include <ros_canbus/time.hpp>
#include <thruster_enitech_driver/JointState.hpp>

namespace thruster_enitech
{
    /** Thruster status as reported by the PDO message */
    struct Status
    {
        /** The reception time of this status */
        base::Time time;
        /** The thruster speed (rad/s) */
        double speed;
        /** The thruster current (A) */
        double current;

        /** Reported overtemperature on the electronics */
        bool overtemp_bg149;
        /** Reported overtemperature on the motor */
        bool overtemp_motor;

        /** Whether the extra gain for motor start is being used or not
         */
        bool start_gain;

        /** Whether the control parameters for air (true) or water (false) are
         * being used
         */
        bool air_parameters;

        /** The control mode
         *
         * It is either RAW (current) or SPEED (rotation speed)
         */
        base::JointState::MODE control_mode;
    };
}

#endif

