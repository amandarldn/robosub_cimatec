/*
 * ThrusterMonitor.cpp
 *
 * 	Created on: 12/07/2021
 * 		Author: diogomartinsac
 */

#include <thruster_enitech_node/ThrusterMonitor.hpp>
#include <thruster_enitech_driver/JointState.hpp>
ThrusterMonitor::ThrusterMonitor()
    : Node("ThrusterMonitor"){
    RCLCPP_INFO(get_logger(), "Thruster Monitor Initialized!");
    loadConfig();
    start();
    initializePublishers();
    initializeSubscribers();
    initializeTimers();
}

ThrusterMonitor::~ThrusterMonitor() {}

bool ThrusterMonitor::loadConfig() {

    int moving_average_time_tmp;
    int error_time_filter_tmp;
    int thruster_timeout_tmp;
    std::string yaml_file_path;

    this->declare_parameter("dead_zone", dead_zone_);
    this->declare_parameter("error_time_filter", error_time_filter_tmp);
    this->declare_parameter("moving_average_time", moving_average_time_tmp);
    this->declare_parameter("tolerance", tolerance_);
    this->declare_parameter("thruster_timeout", thruster_timeout_tmp);
    this->declare_parameter("periodic_update", periodic_update_);
    this->declare_parameter("thruster_node_name", thruster_node_name_);
    this->declare_parameter("samples_threshold", samples_threshold_);
    this->declare_parameter("yaml_file_path",std::string("/ff_ws/src/thruster_enitech_node/config/thruster_monitor.yaml"));

    try {
        dead_zone_ = this->get_parameter("dead_zone").as_int(); 
        error_time_filter_tmp = this->get_parameter("error_time_filter").as_int();
        moving_average_time_tmp = this->get_parameter("moving_average_time").as_int();
        tolerance_ = this->get_parameter("tolerance").as_double(); 
        thruster_timeout_tmp = this->get_parameter("thruster_timeout").as_int();
        periodic_update_ = this->get_parameter("periodic_update").as_int();
        thruster_node_name_ = this->get_parameter("thruster_node_name").as_string();
        samples_threshold_ = this->get_parameter("samples_threshold").as_int();
        yaml_file_path = this->get_parameter("yaml_file_path").as_string();

        try {
            parseYaml(yaml_file_path);
        } catch (const std::exception & e) {
            throw std::runtime_error(
                    std::string("Failed to parse YAML file: ") +
                    e.what());
        }
    
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter: %s", e.what());
        throw std::runtime_error(std::string("Failed to get parameter: ") + e.what());
    }

    error_time_filter_ = base::Time::fromMicroseconds(error_time_filter_tmp);
    moving_average_time_ = base::Time::fromMicroseconds(moving_average_time_tmp);
    thruster_timeout_ = base::Time::fromMicroseconds(thruster_timeout_tmp);

    if (tolerance_ <= 0 || tolerance_ > 1) {
        throw std::runtime_error("The tolerance should be a value inside the interval (0,1]");
    }
    if (samples_threshold_ <= 0) {
        throw std::runtime_error("The samples_threshold should have a positive value.");
    }
    
    try {
        parseYaml(yaml_file_path);
    } catch (const std::exception & e) {
        throw std::runtime_error(
                std::string("Failed to parse YAML file: ") +
                e.what());
    }

    RCLCPP_INFO(get_logger(), "Thrusters Monitor parameters loaded successfully.");

    return true;
}

void ThrusterMonitor::start() {
    received_joint_sample_.resize(thrusters_.size());
    newest_speed_.resize(thrusters_.size());
    newest_joint_sample_.resize(thrusters_.size());
    command_samples_.resize(thrusters_.size());
    status_samples_.resize(thrusters_.size());
    normal_behavior_timestamp_.resize(thrusters_.size());
    started_monitoring_.resize(thrusters_.size());
    last_command_sample_time_.resize(thrusters_.size());
    for (int i = 0; i < thrusters_.size(); i++){
        normal_behavior_timestamp_[i] = base::Time();
        status_samples_[i].clear();
        command_samples_[i].clear();
        started_monitoring_[i] = false;
    }
}

base::Time ThrusterMonitor::getMostRecentTimestamp(const base::Time &_command_timestamp,const base::Time &_status_timestamp) const {
    if (_command_timestamp > _status_timestamp)
        return _command_timestamp;
    else
        return _status_timestamp;
}

void ThrusterMonitor::removeOldSamples(
    std::vector<base::samples::Joints> &_samples,
    const base::Time &_moving_average_time,
    const base::Time &_current_time) const {
    while ( !(_samples.empty()) && ((_current_time - _samples.back().time)
     > _moving_average_time))
        _samples.pop_back();
}

double ThrusterMonitor::calcMovingAverage(
    std::vector<base::samples::Joints> &_samples) const {
    double moving_average = 0;
    for (uint i = 0; i < _samples.size(); i++)
        moving_average += _samples[i].elements[0].speed;

    moving_average /= _samples.size();

    return moving_average;
}

void ThrusterMonitor::outputMonitor(double tracking_error,
    double command_moving_average, double status_moving_average,
    const base::Time &elapsed_time, const base::Time &current_time, int i) {

    thruster_enitech::MonitoringVariables monitor_to_pub;
    monitor_to_pub.tracking_error = tracking_error;
    monitor_to_pub.command_moving_average = command_moving_average;
    monitor_to_pub.command_samples = command_samples_[i].size();
    monitor_to_pub.status_moving_average = status_moving_average;
    monitor_to_pub.status_samples = status_samples_[i].size();
    monitor_to_pub.elapsed_time = elapsed_time;
    monitor_to_pub.time = current_time;
    monitoring_variables_pub_.push_back(create_publisher<thruster_enitech_node_msgs::msg::MonitoringVariables>("thrusters", 10));
}

void ThrusterMonitor::initializePublishers() {
    RCLCPP_INFO(this->get_logger(), "Initializing Thrusters Publishers");
    for (auto thruster : thrusters_) {
        monitoring_variables_pub_.push_back(this->create_publisher<thruster_enitech_node_msgs::msg::MonitoringVariables>(
            thruster.name + "/monitoring_variables", rclcpp::QoS(1).transient_local()));
    }
}

void ThrusterMonitor::jointSamplesCallback(const thruster_enitech_node_msgs::msg::JointSample &msg, const int& i) {
    received_joint_sample_[i] = true;
    newest_joint_sample_[i] = msg;
}

void ThrusterMonitor::initializeSubscribers() {
    RCLCPP_INFO(this->get_logger(), "Initializing Thrusters Subscribers");

    cmd_in_speed_sub_ = this->create_subscription<thruster_enitech_node_msgs::msg::CmdSpeed>(
        thruster_node_name_ + "/cmd_in/speed", 10,
        std::bind(&ThrusterMonitor::speedCommandCallback, this, std::placeholders::_1));

    for (size_t i = 0; i < thrusters_.size(); ++i) {
        std::string topic_name = thruster_node_name_ + "/" + thrusters_[i].name + "/joint_sample";
        joint_samples_sub_.push_back(this->create_subscription<thruster_enitech_node_msgs::msg::JointSample>(
            topic_name, 10,
            [this, i](const thruster_enitech_node_msgs::msg::JointSample::SharedPtr msg) {
                this->jointSamplesCallback(*msg, i);
            }));
    }
}

void ThrusterMonitor::speedCommandCallback(
    const thruster_enitech_node_msgs::msg::CmdSpeed& message_holder) {
    unsigned int msg_size = message_holder.rads_sec.size();
    received_speed_command_ = true;
    newest_speed_.clear();
    for (int i = 0; i < msg_size; i++) {
        newest_speed_[i] = message_holder.rads_sec[i];
        last_command_sample_time_[i] = base::Time::now();
    }
}

void ThrusterMonitor::initializeTimers() {
    RCLCPP_INFO(this->get_logger(), "Initializing Thruster Timers");
    update_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&ThrusterMonitor::update, this)
    );
}
void ThrusterMonitor::update() {
    for (size_t i = 0; i < thrusters_.size(); ++i) {
        base::samples::Joints thrusterStatus;
        thrusterStatus.elements.push_back(base::JointState::Speed(newest_joint_sample_[i].speed));
        double time_in_seconds = static_cast<double>(newest_joint_sample_[i].time.sec) + (newest_joint_sample_[i].time.nanosec / 1e9);
        thrusterStatus.time = base::Time::fromSeconds(time_in_seconds);

        if (received_joint_sample_[i]) {
            if (!thrusterStatus.elements[0].hasSpeed()) {
                RCLCPP_WARN(this->get_logger(), "%s: UNSET_SPEED_FIELD", thrusters_[i].name.c_str());
                return;
            } else if (thrusterStatus.time == base::Time::fromSeconds(0)) {
                RCLCPP_WARN(this->get_logger(), "%s: UNSET_TIME_FIELD", thrusters_[i].name.c_str());
                return;
            }
            status_samples_[i].insert(status_samples_[i].begin(), thrusterStatus);
            
            base::samples::Joints thrusterCommand;
            thrusterCommand.elements.push_back(base::JointState::Speed(newest_speed_[i]));
            
            if (received_speed_command_) {
                if (!thrusterCommand.elements[0].hasSpeed()) {
                    RCLCPP_WARN(this->get_logger(), "%s: UNSET_SPEED_FIELD", thrusters_[i].name.c_str());
                    return;
                }
                thrusterCommand.time = last_command_sample_time_[i];
                command_samples_[i].insert(command_samples_[i].begin(), thrusterCommand);
                received_speed_command_ = false;
            } else {
                thrusterCommand.elements.resize(1);

                if (command_samples_[i].empty() || 
                    (last_command_sample_time_[i] < thrusterStatus.time - thruster_timeout_)) {
                    thrusterCommand.elements[0].speed = 0.0;
                } else {
                    thrusterCommand.elements[0].speed = command_samples_[i].front().elements[0].speed;
                }
                thrusterCommand.time = thrusterStatus.time;
                command_samples_[i].insert(command_samples_[i].begin(), thrusterCommand);
            }

            received_joint_sample_[i] = false;
        }

        if (command_samples_[i].empty() || status_samples_[i].empty()) {
            return;
        }

        base::Time commandTime = command_samples_[i].front().time;
        base::Time statusTime = status_samples_[i].front().time;
        base::Time current_time = getMostRecentTimestamp(commandTime, statusTime);

        if (normal_behavior_timestamp_[i].isNull()) {
            normal_behavior_timestamp_[i] = current_time;
            return;
        }

        removeOldSamples(command_samples_[i], moving_average_time_, current_time);
        removeOldSamples(status_samples_[i], moving_average_time_, current_time);

        if ((command_samples_[i].size() < samples_threshold_) || 
            (status_samples_[i].size() < samples_threshold_)) {
            if (started_monitoring_[i]) {
                RCLCPP_INFO(this->get_logger(), "%s: There are not enough samples.", thrusters_[i].name.c_str());
            }
            return;
        }

        if (!started_monitoring_[i]) {
            RCLCPP_INFO(this->get_logger(), "%s: Monitoring begins with sufficient samples.", thrusters_[i].name.c_str());
        }
        started_monitoring_[i] = true;

        double command_moving_average = calcMovingAverage(command_samples_[i]);
        double status_moving_average = calcMovingAverage(status_samples_[i]);
        double tracking_error = std::numeric_limits<double>::quiet_NaN();
        base::Time elapsed_time;

        if (std::abs(command_moving_average) <= dead_zone_ &&
            std::abs(status_moving_average) <= dead_zone_) {
            normal_behavior_timestamp_[i] = current_time;
            outputMonitor(tracking_error, command_moving_average, status_moving_average,
                          elapsed_time, current_time, i);
            return;
        }

        tracking_error = std::abs((command_moving_average - status_moving_average) /
                                  command_moving_average);

        if (tracking_error <= tolerance_) {
            normal_behavior_timestamp_[i] = current_time;
            outputMonitor(tracking_error, command_moving_average, status_moving_average,
                          elapsed_time, current_time, i);
            return;
        }

        elapsed_time = commandTime - normal_behavior_timestamp_[i];
        outputMonitor(tracking_error, command_moving_average, status_moving_average,
                      elapsed_time, current_time, i);

        if (elapsed_time > error_time_filter_) {
                RCLCPP_WARN(this->get_logger(), "The thruster %s performance is below expected.", thrusters_[i].name.c_str());
            return;
        }
    }
}


void ThrusterMonitor::parseYaml(const std::string & filename) {
    
    YAML::Node config;

    try
    {
        config = YAML::LoadFile(filename);
    }
    catch(const std::exception& e)
    {
        std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
        return;
    }
    
    if(config["ThrusterNode"])
    {
        auto params = config["ThrusterNode"]["ros__parameters"];

        if(params["thrusters"])
        {
            for(const auto &thruster : params["thrusters"])
            {
                thruster_enitech::Thruster thruster_tmp;
                thruster_tmp.name = thruster["name"].as<std::string>();
                thruster_tmp.id = thruster["id"].as<int>();
                thruster_tmp.frame = thruster["frame"].as<std::string>();

                thrusters_.push_back(thruster_tmp);
            }
        } else {
            std::cerr << "No 'thrusters' section found in the YAML file." << std::endl;
        }
    } else {
        std::cerr << "No 'ros__parameters' section found in the YAML file." << std::endl;
    }
}
