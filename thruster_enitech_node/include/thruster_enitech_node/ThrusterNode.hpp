// /*
//  * ThrusterNode.hpp
//  *
//  * 	Created on: 22/07/2021
//  * 		Author: diogomartinsac
//  */

#pragma once

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <thruster_enitech_driver/Protocol.hpp>
#include <thruster_enitech_driver/SDO.hpp>
#include <thruster_enitech_driver/NMT.hpp>
#include <thruster_enitech_driver/Request.hpp>

#include "thruster_enitechTypes.hpp"
#include "Joints.hpp"
#include <yaml-cpp/yaml.h>
#include "thruster_enitech_node_msgs/msg/bg149_temperature.hpp"
#include <thruster_enitech_node_msgs/msg/bg149_temperature_array.hpp>
#include <thruster_enitech_node_msgs/msg/status.hpp>
#include <thruster_enitech_node_msgs/msg/emergency.hpp>
#include <thruster_enitech_node_msgs/msg/joint_sample.hpp>
#include <thruster_enitech_node_msgs/msg/heartbeat.hpp>
#include <thruster_enitech_node_msgs/msg/cmd_speed.hpp>
#include <thruster_enitech_node_msgs/msg/cmd_raw.hpp>

class ThrusterNode : public rclcpp::Node 
{
 private:
    /**
     * @brief Canbus driver pointer.
     *msgs_thruster_node
     */
    canbus::Driver* driver_;

    /**
     * @brief List of all declared thrusters.
     *
     */
    std::vector<thruster_enitech::Thruster> thrusters_;

    /**
     * @brief The amount of time we wait to get a ready-to-use thruster during configuration. The default is 2s.
     *
     */
    int reset_timeout_;

    /**
     * @brief The amount of time we wait for the thrusters to start. The default is 1s.
     *
     */
    int start_timeout_;

    /**
     * @brief The amount of time we wait for the thrusters to stop. The default is 0.5s.
     *
     */
    int stop_timeout_;

    /**
     * @brief The amount of time we wait for the node to ack a register read/write. The default is 0.5s
     *
     */
    int sdo_timeout_;

    /**
     * @brief The heartbeat period (between 0 and 65535ms). Defaults to 2s.
     *
     */
    int heartbeat_period_;

    /**
     * @brief The period at which the device sends status information (between 0 and 65535ms). Defaults to 0.1s.
     *
     */
    int update_period_;

    /**
     * @brief Writing period of temperature on the output port (between 0 and 65535ms). Defaults to 1s.
     *
     */
    int temperature_period_;

    /**
     * @brief The period at which the node publish new messages and send signals to thrusters. Defaults to 0.1s.
     *
     */
    int periodic_update_;

    /**
     * @brief CAN network interface name.
     *
     */
    std::string device_;

    /**
     * @brief Representation of the low-level CAN protocol for a given thruster. It handles core messages (PDO, emergency and hearbeat), and maintains the
     * known state of the thruster (as reported by the heartbeats).
     *
     */
    std::vector<thruster_enitech::Protocol> protocol_;

    /**
     * @brief The time without thruster status after which the component goes into IO_TIMEOUT.
     *
     */
    base::Time status_timeout_;

    /**
     * @brief Vector containing the last status message timestamp of each thruster.
     *
     */
    std::vector<base::Time> last_status_;

    /**
     * @brief Active sampling period.
     *
     */
    base::Time const poll_period_;

    /**
     * @brief Thruster status that is provided by the device. Contains timestamp, speed (rad/s) and current (A).
     *
     */
    std::vector<base::samples::Joints> joints_;

    /**
     * @brief Contains probe info. Contains probe id, temperature in degrees and timestamp. Each thruster has 3 probes.
     *
     */
    std::vector<thruster_enitech_node_msgs::msg::BG149Temperature> bg149_temperature_;

    /**
     * @brief Contains probes info of all declared thrusters.
     *
     */
    std::vector<thruster_enitech_node_msgs::msg::BG149TemperatureArray> bg149_temperature_array_;

    /**
     * @brief Contains the newest received speed command of each thruster.
     *
     */
    std::vector<float> newest_speed_;

    /**
     * @brief Contains the newest received raw command of each thruster.
     *
     */
    std::vector<float> newest_raw_;

    /**
     * @brief This flag holds a true value if a new speed command was received.
     *
     */
    bool received_speed_command_;

    /**
     * @brief This flag holds a true value if a new raw command was received.
     *
     */
    bool received_raw_command_;

    /**
     * @brief Subscribes to cmd_in/speed topic and handle its messages.
     *
     */
    rclcpp::Subscription<thruster_enitech_node_msgs::msg::CmdSpeed>::SharedPtr cmd_in_speed_sub_;
    /**
     * @brief Subscribes to cmd_in/raw topic and handle its messages.
     *
     */
    rclcpp::Subscription<thruster_enitech_node_msgs::msg::CmdRaw>::SharedPtr cmd_in_raw_sub_;
    /**
     * @brief Publishes joint samples of the thrusters.
     *
     */
    std::vector<rclcpp::Publisher<thruster_enitech_node_msgs::msg::JointSample>::SharedPtr> joint_samples_pub_;

    /**
     * @brief Publishes the hearbeat of the thrusters.
     *
     */
    std::vector<rclcpp::Publisher<thruster_enitech_node_msgs::msg::Heartbeat>::SharedPtr> heartbeat_pub_;

    /**
     * @brief Publishes the status of the thrusters.
     *
     */
    std::vector<rclcpp::Publisher<thruster_enitech_node_msgs::msg::Status>::SharedPtr> status_pub_;

    /**
     * @brief Publishes the emergency messages of the thrusters.
     *
     */
    std::vector<rclcpp::Publisher<thruster_enitech_node_msgs::msg::Emergency>::SharedPtr> emergency_pub_;

    /**
     * @brief Publishes the temperature of all probes of the thrusters.
     *
     */
    std::vector<rclcpp::Publisher<thruster_enitech_node_msgs::msg::BG149TemperatureArray>::SharedPtr> temperature_pub_;

    /**
     * @brief Request the status of all thrusters, publishes its info and send actuation signals based on a given time interval.
     *
     */
    rclcpp::TimerBase::SharedPtr update_;

 public:
    /**
     * @brief Construct a new Thruster Node object
     *
     * @param nodehandle
     */
     ThrusterNode();

    /**
     * @brief Destroy the Thruster Node object
     *
     */
    virtual ~ThrusterNode();

 private:
    /**
     * @brief
     *
     * @param request Request sent to the device. Protocol implements the request generation.
     * @param timeout Declared timeout.
     * @return true Request successful.
     * @return false IO timeout.
     */
    bool processRequest(thruster_enitech::Request& request,
                        base::Time const& timeout);

    /**
     * @brief
     *
     * @param probe_idx Probe id - 0x01, 0x02 or 0x03.
     * @param i Thruster identifier
     * @return true Temperature was readed.
     * @return false Couldn't read the temperature.
     */
    bool readTemperature(int probe_idx, int i);

    /**
     * @brief Load ROS parameters. Remember to load "*.yaml" alongside the node in the launch file.
     *
     * @return true Successfully loaded thrusters parameters.
     * @return false Thruster parameters failed to load.
     */
    bool loadConfig();

    /**
     * @brief Initialize thrusters.
     *
     * @return true
     * @return false
     */
    bool initializeThruster();

    /**
     * @brief Stop all declared thrusters.
     *
     */
    void stop();

    /**
     * @brief Setup and initialize publishers.
     *
     */
    void initializePublishers();

    /**
     * @brief Setup and initialize subscribers.
     *
     */
    void initializeSubscribers();

    /**
     * @brief Setup and initialize timers.
     *
     */
    void initializeTimers();

    /**
     * @brief Speed command topic callback. Handles all incoming messages.
     *
     * @param message_holder Received speed message.
     */
    void speedCommandCallback(const thruster_enitech_node_msgs::msg::CmdSpeed::SharedPtr message_holder);

    /**
     * @brief Raw command topic callback. Handles all incoming messages.
     *
     * @param message_holder Received raw message.
     */
    void rawCommandCallback(const thruster_enitech_node_msgs::msg::CmdRaw::SharedPtr
                                                        message_holder);

    /**
     * @brief Request the status of all thrusters, publishes its info and send actuation signals based on a given time interval.
     *
     */
    void update();

    /**
     * @brief Parse thrusters info from Yaml to a vector of thrusters.
     *
     * @param thruster_list
     */
    void parseYaml(const std::string & filename);
};
