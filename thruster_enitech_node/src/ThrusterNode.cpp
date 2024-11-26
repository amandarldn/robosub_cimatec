// /*
//  * ThrusterNode.cpp
//  *
//  * 	Created on: 10/08/2021
//  * 		Author: diogomartinsac
//  */

#include <thruster_enitech_node/ThrusterNode.hpp>

ThrusterNode::ThrusterNode() : Node("ThrusterNode"){
    RCLCPP_INFO(this->get_logger(), "Thruster Node Initialized!");
    loadConfig();
    initializeThruster();
    initializePublishers();
    initializeSubscribers();
    initializeTimers();
}

ThrusterNode::~ThrusterNode() 
{
    stop();
}


bool ThrusterNode::loadConfig() {

    int status_timeout_cfg_;
    std::string yaml_file_path;

    this->declare_parameter("device", device_);
    this->declare_parameter("periodic_update", periodic_update_);
    this->declare_parameter("reset_timeout", reset_timeout_);
    this->declare_parameter("start_timeout", start_timeout_);
    this->declare_parameter("stop_timeout", stop_timeout_);
    this->declare_parameter("sdo_timeout", sdo_timeout_);
    this->declare_parameter("heartbeat_period", heartbeat_period_);
    this->declare_parameter("status_timeout", status_timeout_cfg_);
    this->declare_parameter("update_period", update_period_);
    this->declare_parameter("temperature_period", temperature_period_);
    this->declare_parameter("yaml_file_path",std::string("/ff_ws/src/thruster_enitech_node/config/thruster_node.yaml"));

    try {
        device_ = this->get_parameter("device").as_string();
        periodic_update_ = this->get_parameter("periodic_update").as_int();
        sdo_timeout_ = this->get_parameter("sdo_timeout").as_int();
        reset_timeout_ = this->get_parameter("reset_timeout").as_int();
        start_timeout_ = this->get_parameter("start_timeout").as_int();
        stop_timeout_ = this->get_parameter("stop_timeout").as_int();
        start_timeout_ = this->get_parameter("start_timeout").as_int();
        heartbeat_period_ = this->get_parameter("heartbeat_period").as_int();
        update_period_ = this->get_parameter("update_period").as_int();
        temperature_period_ = this->get_parameter("temperature_period").as_int();
        yaml_file_path = this->get_parameter("yaml_file_path").as_string();

        try
        {
            parseYaml(yaml_file_path);
        }
        catch(const std::exception& e)
        {
            throw std::runtime_error(
                    std::string("Failed to parse YAML file: ") +
                    e.what());        
        }
        

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter: %s", e.what());
        throw std::runtime_error(std::string("Failed to get parameter: ") + e.what());
    }
        
    bg149_temperature_.resize(thrusters_.size());
    joints_.resize(thrusters_.size());
    bg149_temperature_array_.resize(thrusters_.size());
    RCLCPP_INFO(this->get_logger(), "Thrusters parameters loaded successfully.");

    driver_ = canbus::openCanDevice(device_, "SOCKET");
    if (!driver_) {
        throw std::runtime_error(
            "Failed to open the CAN device.");
    }
    if (!driver_->reset()) {
        throw std::runtime_error(
            "Failed to reset the CAN device.");
    }
    for (thruster_enitech::Thruster& thruster : thrusters_) {
        protocol_.push_back(thruster_enitech::Protocol(thruster.id));
        int protocol_size = protocol_.size();

        joints_[protocol_size - 1].elements.resize(1);

        thruster_enitech::SDO::WriteHeartbeatPeriod heartbeat(
            protocol_[protocol_size - 1], base::Time::fromMicroseconds(heartbeat_period_));
        if (!processRequest(heartbeat, base::Time::fromMicroseconds(sdo_timeout_)))
            return false;

        thruster_enitech::NMT::Reset reset(protocol_[protocol_size - 1]);

        if (!processRequest(reset, base::Time::fromMicroseconds(reset_timeout_)))
            return false;

        thruster_enitech::SDO::WriteUpdatePeriod update(
            protocol_[protocol_size - 1], base::Time::fromMicroseconds(update_period_));
        if (!processRequest(update, base::Time::fromMicroseconds(sdo_timeout_)))
            return false;
    }
    return true;
}

bool ThrusterNode::initializeThruster() {
    RCLCPP_INFO(this->get_logger(), "Initializing Thrusters");
    for (thruster_enitech::Protocol& protocol : protocol_) {
        thruster_enitech::NMT::Start start(protocol);
        if (!processRequest(start, status_timeout_))
            return false;
        last_status_.push_back(base::Time::now());
    }
    return true;
}

void ThrusterNode::initializePublishers() {
    RCLCPP_INFO(this->get_logger(), "Initializing Thrusters Publishers");
    for (int i = 0; i < thrusters_.size(); i++) {
        joint_samples_pub_.push_back(this->create_publisher<thruster_enitech_node_msgs::msg::JointSample>(
            thrusters_[i].name + "/joint_sample", 10));

        heartbeat_pub_.push_back(this->create_publisher<thruster_enitech_node_msgs::msg::Heartbeat>(
            thrusters_[i].name + "/heartbeat", 10));

        status_pub_.push_back(this->create_publisher<thruster_enitech_node_msgs::msg::Status>(
        thrusters_[i].name + "/status", 10));

        emergency_pub_.push_back(this->create_publisher<thruster_enitech_node_msgs::msg::Emergency>(
            thrusters_[i].name + "/emergency", 10));

        temperature_pub_.push_back(this->create_publisher<thruster_enitech_node_msgs::msg::BG149TemperatureArray>(
            thrusters_[i].name + "/temperature", 10));                                          
    }
}

void ThrusterNode::initializeSubscribers() {
    RCLCPP_INFO(this->get_logger(), "Initializing Thrusters Subscribers");

    cmd_in_speed_sub_ = this->create_subscription<thruster_enitech_node_msgs::msg::CmdSpeed>(
            "cmd_in/speed", 1 ,std::bind(&ThrusterNode::speedCommandCallback, this, std::placeholders::_1));

    cmd_in_raw_sub_ = this->create_subscription<thruster_enitech_node_msgs::msg::CmdRaw>(
        "cmd_in/raw", 1, 
        std::bind(&ThrusterNode::rawCommandCallback, this, std::placeholders::_1));
}

void ThrusterNode::initializeTimers()
{
    RCLCPP_INFO(this->get_logger(), "Initializing Thruster Timers");

    update_ = this->create_wall_timer(
        std::chrono::milliseconds(periodic_update_ / 1000),
        std::bind(&ThrusterNode::update, this)
    );
}

void ThrusterNode::speedCommandCallback(const thruster_enitech_node_msgs::msg::CmdSpeed::SharedPtr message_holder)
{
    RCLCPP_INFO(this->get_logger(), "Received Thruster Speed Command");

    unsigned int msg_size = message_holder->rads_sec.size();
    if (msg_size == thrusters_.size()) {
        received_speed_command_ = true;
        newest_speed_.clear();
        newest_speed_.resize(msg_size);
        
        for (size_t i = 0; i < msg_size; ++i) {
            newest_speed_[i] = message_holder->rads_sec[i];
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Received command doesn't match the number of declared thrusters."
                                         " Correct usage: rostopic pub /NodeName/cmd_in/speed thruster_enitech_node/CmdSpeed"
                                         " \"rads_sec: [cmd_1, cmd_2, cmd_3, ...]\"");
    }
}

void ThrusterNode::rawCommandCallback(const thruster_enitech_node_msgs::msg::CmdRaw::SharedPtr message_holder)
{
    RCLCPP_INFO(this->get_logger(), "Received Thruster Raw Command");

    unsigned int msg_size = message_holder->pwm.size();
    if (msg_size == thrusters_.size()) {
        received_raw_command_ = true;
        newest_raw_.clear();
        newest_raw_.resize(msg_size);

        for (size_t i = 0; i < msg_size; ++i) {
            newest_raw_[i] = message_holder->pwm[i];
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Received command doesn't match the number of declared thrusters."
                                         " Correct usage: rostopic pub /NodeName/cmd_in/raw thruster_enitech_node/CmdRaw"
                                         " \"pwm: [cmd_1, cmd_2, cmd_3, ...]\"");
    }
}

void ThrusterNode::stop() {
    for (thruster_enitech::Protocol& protocol : protocol_) {
        thruster_enitech::NMT::Stop stop(protocol);
        processRequest(stop, base::Time::fromMicroseconds(stop_timeout_));
    }
}


void ThrusterNode::update()
{
    for (int i = 0; i < thrusters_.size(); i++) {
        canbus::Message msg;
        if (driver_->readCanMsg(msg)) {
            thruster_enitech::MSG_TYPE msg_type = protocol_[i].update(msg);
            if (msg_type == thruster_enitech::MSG_NONE) {
                return;
            } else if (msg_type == thruster_enitech::MSG_STATUS) {
                thruster_enitech::Status status = protocol_[i].getLastStatus();
                joints_[i].time = status.time;
                last_status_[i] = status.time;
                joints_[i].elements[0].speed = status.speed;
                joints_[i].elements[0].raw = status.current;

                auto status_to_pub = thruster_enitech_node_msgs::msg::Status();
                status_to_pub.time = rclcpp::Time(status.time.toSeconds());
                status_to_pub.speed = status.speed;
                status_to_pub.current = status.current;
                status_to_pub.overtemp_bg149 = status.overtemp_bg149;
                status_to_pub.overtemp_motor = status.overtemp_motor;
                status_to_pub.start_gain = status.start_gain;
                status_to_pub.air_parameters = status.air_parameters;
                status_to_pub.mode = status.control_mode;
                status_pub_[i]->publish(status_to_pub);

                auto sample_to_pub = thruster_enitech_node_msgs::msg::JointSample();
                sample_to_pub.time = rclcpp::Time(status.time.toSeconds());
                sample_to_pub.speed = status.speed;
                sample_to_pub.current = status.current;
                joint_samples_pub_[i]->publish(sample_to_pub);

            } else if (msg_type == thruster_enitech::MSG_EMERGENCY) {
                thruster_enitech::Emergency emergency = protocol_[i].getLastEmergency();
                auto emergency_to_pub = thruster_enitech_node_msgs::msg::Emergency();
                emergency_to_pub.time = rclcpp::Time(emergency.time.toSeconds());
                emergency_to_pub.overtemp_motor = emergency.overtemp_motor;
                emergency_to_pub.overtemp_bg149 = emergency.overtemp_bg149;
                emergency_to_pub.fault_free = emergency.fault_free;
                emergency_to_pub.hardware_error = emergency.hardware_error;
                emergency_to_pub.sensor_error = emergency.sensor_error;
                emergency_to_pub.data_error = emergency.data_error;
                emergency_pub_[i]->publish(emergency_to_pub);
            }

            base::Time temperature_period = base::Time::fromMicroseconds(temperature_period_);
            double bg149_temperature_in_seconds = static_cast<double>(bg149_temperature_[i].last_update.sec) + 
            (bg149_temperature_[i].last_update.nanosec / 1e9);            
            if (!temperature_period.isNull() && (base::Time::now() -
                base::Time::fromSeconds(bg149_temperature_in_seconds)) >= temperature_period) {
                int probe_idx = ((bg149_temperature_[i].probe_id + 1) % 3) + 1;
                if (!readTemperature(probe_idx, i)) {
                    RCLCPP_INFO(this->get_logger(), "Temperature could not be read.");
                    return;
                }
            }
        }

        if (received_speed_command_) {
            driver_->write(protocol_[i].makeCommand(base::JointState::Speed(newest_speed_[i])));
            received_speed_command_ = false;
        } else if (received_raw_command_) {
            driver_->write(protocol_[i].makeCommand(base::JointState::Raw(newest_raw_[i])));
            received_raw_command_ = false;
        }

        base::Time lastHeartbeat = protocol_[i].getLastHeartbeat();
        auto heartbeat_to_pub = thruster_enitech_node_msgs::msg::Heartbeat();
        heartbeat_to_pub.last_heartbeat = rclcpp::Time(lastHeartbeat.toSeconds());

        heartbeat_pub_[i]->publish(heartbeat_to_pub);
        if (last_status_[i] + status_timeout_ < base::Time::now()) {
            throw std::runtime_error("IO_TIMEOUT");
        }
    }
}


bool ThrusterNode::processRequest(thruster_enitech::Request& request,
                                            base::Time const& timeout) {
    driver_->write(request.message);
    canbus::Message msg;
    base::Time start = base::Time::now();
    do {
        if (driver_->readCanMsg(msg))
            if (request.update(msg))
                return true;
        usleep(poll_period_.toMilliseconds() * 1000);
    } while (base::Time::now() - start < timeout);

    throw std::runtime_error("IO_TIMEOUT");
    return false;
}

bool ThrusterNode::readTemperature(int probe_idx, int i)
{
    thruster_enitech::SDO::ReadBG149Temperature read_temp(protocol_[i], probe_idx);
    
    if (!processRequest(read_temp, base::Time::fromMicroseconds(sdo_timeout_))) {
        RCLCPP_ERROR(this->get_logger(), "Failed to process temperature request.");
        return false;
    }

    bg149_temperature_[i].probe_id = probe_idx;
    bg149_temperature_[i].temp_in_degrees = read_temp.value;
    bg149_temperature_[i].last_update = rclcpp::Time(read_temp.time.toSeconds());

    bg149_temperature_array_[i].probes[probe_idx - 1] = bg149_temperature_[i];
    temperature_pub_[i]->publish(bg149_temperature_array_[i]);

    RCLCPP_INFO(this->get_logger(), "Temperature read successfully for probe %d, thruster %d", probe_idx, i);

    return true;
}

void ThrusterNode::parseYaml(const std::string & filename) {
    
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
