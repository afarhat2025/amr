#include "amr_node.h"
#include <QDebug>
#include <cstdlib>
#include <string>
#include <QTimer>
#include "std_msgs/msg/float64.hpp"
#include <fstream>
#include <streambuf>


AmrNode::AmrNode(const std::string &robot_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node("amr_node", options), robot_name_(robot_name)
{

    error_sub_ = this->create_subscription<amr_v4_msgs_srvs::msg::Error>(
        "system_hardware_error", 10, 
        std::bind(&AmrNode::error_callback, this, std::placeholders::_1));

    battery_sub_ = this->create_subscription<amr_v4_msgs_srvs::msg::Battery>(
            "battery_state", 10,
            std::bind(&AmrNode::battery_callback, this, std::placeholders::_1));

    motor_sub_ = this->create_subscription<amr_v4_msgs_srvs::msg::Motor>(
            "talonFX_motors/status", 10,
            std::bind(&AmrNode::motor_callback, this, std::placeholders::_1));

    robot_sub_ = this->create_subscription<amr_v4_msgs_srvs::msg::Robot>(
            "diagnostic", 10,
            std::bind(&AmrNode::robot_callback, this, std::placeholders::_1));

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "e_stop", 10,
            std::bind(&AmrNode::estop_callback, this, std::placeholders::_1));
        
    mode_pub_ = this->create_publisher<amr_v4_msgs_srvs::msg::Mode>("mode", 10);

    pin_pub_ = this->create_publisher<amr_v4_msgs_srvs::msg::Pin>("pin_cmd", 10);


     // Set up a timer to call retrieveStationIdFromMongoDB periodically
    QTimer *mongoDbTimer = new QTimer();
    connect(mongoDbTimer, &QTimer::timeout, [this]() {
        retrieveStationIdFromMongoDB();
    });
    

    mongoDbTimer->start(5000);  // Check every 5 seconds
}

rclcpp::Node::SharedPtr AmrNode::get_raw_node() {
    return self_node_ptr_;
}

std::string AmrNode::get_node_name() const {
    return this->get_name();
}

std::map<std::string, std::vector<std::string>> AmrNode::get_topic_names_and_types() const {
    return rclcpp::Node::get_topic_names_and_types();
}

void AmrNode::setSelfPtr(std::shared_ptr<AmrNode> self) {
    self_node_ptr_ = self;
}


void AmrNode::createChargingPublisher()
{
    if (!charging_pub_) {
        std::string charging_topic = "/" + robot_name_ + "/charger_cmd";
        charging_pub_ = this->create_publisher<amr_v4_msgs_srvs::msg::Charging>(charging_topic, 10);
    }
}

void AmrNode::destroyChargingPublisher()
{
    if (charging_pub_) {
        charging_pub_.reset(); // Destroy the publisher
    }
}

// Function to retrieve the robot's unique identifier
std::string AmrNode::getRobotId() {
    return robot_name_;  // Use the robot_name_ member variable
}

void AmrNode::retrieveStationIdFromMongoDB() {
    try {
        // Initialize MongoDB driver once
        static mongocxx::instance instance{};
        
        // Connect to MongoDB with the provided credentials and host details
        // qDebug() << "Connecting to MongoDB...";
        mongocxx::client client{mongocxx::uri{"mongodb://mrumel:56.583fudge@10.10.1.45:27017/?authSource=admin"}};
        auto db = client["project6"];               // Database name
        auto collection = db["AMR"];                // Collection name

        // Retrieve the robot's unique identifier
        std::string robot_name = getRobotId();
        int robot_id = std::stoi(robot_name.substr(robot_name.find_last_of('_') + 1));  // Assuming the ID is part of the name

        // Query MongoDB for the document where "amr" matches this robot's ID
        // qDebug() << "Querying MongoDB for robot ID:" << robot_id;
        auto document = collection.find_one(bsoncxx::builder::basic::make_document(
            bsoncxx::builder::basic::kvp("amr", robot_id)));

        // If a document is found, retrieve the station_curr and emit the signal
        if (document) {
            auto element = document->view()["station_curr"];
            auto element_1 = document->view()["mission_type"];
            std::string station_id;
            std::string mission_type;
            if (element.type() == bsoncxx::type::k_string) {
                station_id = std::string(element.get_string().value);
                // qDebug() << "Station ID retrieved from MongoDB:" << QString::fromStdString(station_id);
                mission_type = std::string(element_1.get_string().value);

                if (station_id.length() > 0 ){
                    emit changedStation(QString::fromStdString(station_id));
                } else {
                    emit changedStation(QString::fromStdString(mission_type));
                }
            } 
        } 
    } catch (const std::exception &e) {
        // qDebug() << "Failed to retrieve station ID from MongoDB:" << e.what();
    }
}


void AmrNode::error_callback(const amr_v4_msgs_srvs::msg::Error::SharedPtr msg)
{
    slam_lidar = msg->slam_lidar;
    estop_lidar = msg->estop_lidar;
    camera = msg->camera;

    emit(changedError(slam_lidar, estop_lidar, camera));
}
void AmrNode::battery_callback(const amr_v4_msgs_srvs::msg::Battery::SharedPtr msg)
{
    soc = msg->soc;
    voltage = msg->voltage;
    current = msg->current;
    temperature = QString::fromStdString(msg->temp);

    emit changedBattery(voltage, current, soc,temperature);
}

void AmrNode::motor_callback(const amr_v4_msgs_srvs::msg::Motor::SharedPtr msg)
{
    output_current_right = msg->output_current_right.c_str();
    output_current_left = msg->output_current_left.c_str();
    output_current_pin = msg->output_current_pin.c_str();
    error_right = msg->error_right.c_str();
    error_left = msg->error_left.c_str();
    error_pin = msg->error_pin.c_str();

    emit changedMotor(output_current_right, output_current_left, output_current_pin,
    error_right, error_left, error_pin);
}

void AmrNode::robot_callback(const amr_v4_msgs_srvs::msg::Robot::SharedPtr msg)
{
    status = QString::fromStdString(msg->robot_localization_status);  // Convert std::string to QString
    emit changedRobot(status);
}
void AmrNode::estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    estop = msg->data;
    emit changedEstop(estop);
}


void AmrNode::mode_callback(const bool msg)
{
    auto message = amr_v4_msgs_srvs::msg::Mode();
    message.mode = msg;
    mode_pub_->publish(message);
}

void AmrNode::pin_callback(const bool msg)
{
    auto message = amr_v4_msgs_srvs::msg::Pin();
    message.pin_command = msg;
    pin_pub_->publish(message);
}

void AmrNode::publishChargingMessage(bool start_charging_main)
{
    if (charging_pub_) {
        auto charging_msg = amr_v4_msgs_srvs::msg::Charging();
        charging_msg.start_charging_main = start_charging_main;
        charging_pub_->publish(charging_msg);
        qDebug() << "Charging message published: start_charging_main =" << start_charging_main;
    } else {
        qDebug() << "Charging publisher is not created.";
    }
}
void AmrNode::updateChargingState(bool charging_active) {
    if (charging_active) {
        qDebug() << "Charging started. Monitoring battery updates.";
    
    } else {
        qDebug() << "Charging stopped.";
    }
}


std::string runCommand(const std::string& cmd) {
    std::array<char, 4096> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}
    
