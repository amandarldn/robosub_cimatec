// /*
//  * ThrusterMonitor.hpp
//  *
//  * 	Created on: 29/07/2021
//  * 		Author: diogomartinsac
//  */

#pragma once

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include "Joints.hpp"
#include "thruster_enitechTypes.hpp"
#include <yaml-cpp/yaml.h>
#include <thruster_enitech_driver/JointState.hpp>

#include <thruster_enitech_node_msgs/msg/monitoring_variables.hpp>
#include <thruster_enitech_node_msgs/msg/joint_sample.hpp>
#include <thruster_enitech_node_msgs/msg/cmd_speed.hpp>

class ThrusterMonitor : public rclcpp::Node 
{
 protected:
   /**
    * @brief Time horizon for the moving average filter in microseconds.
    *
    */
    base::Time moving_average_time_;
    /**
     * @brief Time that the component waits in microseconds until warns "Performance below expected". This happens if the tracking error stays above the specified tolerance for more than the given period.
     *
     */
    base::Time error_time_filter_;
    /**
     * @brief Tolerance for the thruster monitoring between 0 and 1.
     *
     */
    double tolerance_;

    /**
     * @brief Contains status samples of the thrusters.
     *
     */
    std::vector<std::vector<base::samples::Joints>> status_samples_;

    /**
     * @brief Contains speed command samples.
     *
     */
    std::vector<std::vector<base::samples::Joints>> command_samples_;

    /**
     * @brief Normal behavior timestamp vector.
     *
     */
    std::vector<base::Time> normal_behavior_timestamp_;

    /**
     * @brief  Time tolerance for thruster command input arrival.
     *
     */
    base::Time thruster_timeout_;

    /**
     * @brief Timestamp of the last received command sample.
     *
     */
    std::vector<base::Time> last_command_sample_time_;

    /**
     * @brief Monitoring flag. Holds true if thruster monitoring started.
     *
     */
    std::vector<bool> started_monitoring_;

    /**
     * @brief Necessary number of samples to start the monitoring process.
     *
     */
    int samples_threshold_;
    /**
     * @brief Remove old command samples.
     *
     * @param _samples
     * @param _moving_average_time
     * @param current_time
     */
    void removeOldSamples(std::vector<base::samples::Joints> &_samples,
            const base::Time &_moving_average_time,
            const base::Time &current_time) const;

    /**
     * @brief Calculates commands moving average.
     *
     * @param _samples
     * @return double
     */
    double calcMovingAverage(std::vector<base::samples::Joints> &_samples) const;

    /**
     * @brief Get the most recent timestamp object.
     *
     * @param _command_timestamp
     * @param _status_timestamp
     * @return base::Time
     */
    base::Time getMostRecentTimestamp(const base::Time &_command_timestamp,
            const base::Time &_status_timestamp) const;

    /**
     * @brief Publishes monitoring variables into "NodeName/thrusterName/monitoring_variables" topic.
     *
     * @param tracking_error
     * @param command_moving_average
     * @param status_moving_average
     * @param elapsed_time
     * @param current_time
     * @param i Thruster index.
     */
    void outputMonitor(double tracking_error,
            double command_moving_average, double status_moving_average,
            const base::Time &elapsed_time, const base::Time &current_time, int i);

 private:
     
    /**
     * @brief List of all declared thrusters.
     *
     */
    std::vector<thruster_enitech::Thruster> thrusters_;

    /**
     * @brief Minimal absolute value from where the tracking error is calculated. This is needed because for small rotation speed values, the dynamic changes really fast and it results on a peak of tracking error.
     *
     */
    int dead_zone_;

    /**
     * @brief The period at which the update method is called.
     *
     */
    int periodic_update_;

    /**
     * @brief Holds the Thruster node name.
     *
     */
    std::string thruster_node_name_;

    /**
     * @brief Speed command flag. Holds true if a new speed command was received.
     *
     */
    bool received_speed_command_ = false;

    /**
     * @brief Joint sample flag. Holds true if a new joint sample was received.
     *
     */
    std::vector<bool> received_joint_sample_;

    /**
     * @brief Contains the latest speed command values.
     *
     */
    std::vector<float> newest_speed_;

    /**
     * @brief Contains the latest joint sample values.
     *
     */
    std::vector<thruster_enitech_node_msgs::msg::JointSample> newest_joint_sample_;
    /**
     * @brief Subscribes to "NodeName/ThrusterName/cmd_in/speed" topic.
     *
     */
    rclcpp::Subscription<thruster_enitech_node_msgs::msg::CmdSpeed>::SharedPtr cmd_in_speed_sub_;

    /**
     * @brief Subscribes to "NodeName/ThrusterName/joint_samples" topic.
     *
     */
    std::vector<rclcpp::Subscription<thruster_enitech_node_msgs::msg::JointSample>::SharedPtr> joint_samples_sub_;
    /**
     * @brief Publishes monitoring variables.
     *
     */
    std::vector<rclcpp::Publisher<thruster_enitech_node_msgs::msg::MonitoringVariables>::SharedPtr> monitoring_variables_pub_;
    /**
     * @brief Update method timer.
     *
     */
    rclcpp::TimerBase::SharedPtr update_;

 public:
    /**
     * @brief Construct a new Thruster Monitor object
     *
     */
    explicit ThrusterMonitor();

    /**
     * @brief Destroy the Thruster Monitor object
     *
     */
    virtual ~ThrusterMonitor();

 private:

     /**
     * @brief Load ROS parameters. Remember to load "*.yaml" alongside the node in the launch file.
     *
     * @return true Successfully loaded parameters.
     * @return false Parameters failed to load.
     */
    bool loadConfig();

    /**
     * @brief Resize and initialize variables.
     *
     */
    void start();

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
    void speedCommandCallback(const thruster_enitech_node_msgs::msg::CmdSpeed&
                                                        message_holder);
    /**
     * @brief Joint samples topic callback. Handles all incoming messages.
     *
     * @param msg Received message.
     * @param i Thruster index.
     */
    void jointSamplesCallback(const thruster_enitech_node_msgs::msg::JointSample &msg, const int& i);

    /**
     * @brief Analize commands and status samples, publishes monitoring variables and warns if thruster performance is below expected.
     *
     */
    void update();

    /**
     * @brief Parse thrusters info from Yaml to a vector of thrusters.
     *
     *      
    */
    void parseYaml(const std::string & filename);
    

};
