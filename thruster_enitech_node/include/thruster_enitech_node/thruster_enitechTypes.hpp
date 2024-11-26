#ifndef ROS_THRUSTER_ENITECH_NODE_INCLUDE_ROS_THRUSTER_ENITECH_NODE_THRUSTER_ENITECHTYPES_HPP_
#define ROS_THRUSTER_ENITECH_NODE_INCLUDE_ROS_THRUSTER_ENITECH_NODE_THRUSTER_ENITECHTYPES_HPP_

#include <string>
#include <ros_canbus/time.hpp>


namespace thruster_enitech {

struct Thruster {
    std::string name;
    int id;
    std::string frame;
};

struct MonitoringVariables {
    double command_moving_average;
    uint command_samples;
    double status_moving_average;
    uint status_samples;
    double tracking_error;
    base::Time elapsed_time;
    base::Time time;
};

struct BG149Temperature {
    BG149Temperature() :
        probe_idx(1),
        temp_in_degrees(0) {}

    int probe_idx;
    int temp_in_degrees;
    base::Time last_update;
};

};  // namespace thruster_enitech

#endif  // ROS_THRUSTER_ENITECH_NODE_INCLUDE_ROS_THRUSTER_ENITECH_NODE_THRUSTER_ENITECHTYPES_HPP_

