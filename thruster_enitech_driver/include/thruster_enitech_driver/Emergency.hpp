#ifndef THRUSTER_ENITECH_EMERGENCY_HPP
#define THRUSTER_ENITECH_EMERGENCY_HPP

#include <ros_canbus/time.hpp>

namespace thruster_enitech
{
    struct Emergency
    {
        base::Time time;

        bool overtemp_motor;
        bool overtemp_bg149;

        bool fault_free;
        bool hardware_error;
        bool sensor_error;
        bool data_error;

        Emergency()
            : overtemp_motor(false)
            , overtemp_bg149(false)
            , fault_free(false)
            , hardware_error(false)
            , sensor_error(false)
            , data_error(false) {}
    };
}

#endif
