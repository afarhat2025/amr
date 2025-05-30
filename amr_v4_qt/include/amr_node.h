#ifndef BATTERY_NODE_H
#define BATTERY_NODE_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "amr_v4_msgs_srvs/msg/motor.hpp"
#include "amr_v4_msgs_srvs/msg/pin.hpp"
#include "amr_v4_msgs_srvs/msg/mode.hpp"
#include "amr_v4_msgs_srvs/msg/robot.hpp"
#include "amr_v4_msgs_srvs/msg/error.hpp"
#include "amr_v4_msgs_srvs/msg/charging.hpp"
#include "amr_v4_msgs_srvs/msg/battery.hpp"
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <bsoncxx/json.hpp>
#include <string>
#include <QObject>
#include <QString>
#include "std_msgs/msg/float64.hpp"
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>



using std::placeholders::_1;

class AmrNode : public QObject, public rclcpp::Node, public rviz_common::ros_integration::RosNodeAbstractionIface, public std::enable_shared_from_this<AmrNode>
{
    Q_OBJECT

public:
    explicit AmrNode(const std::string &robot_name, const rclcpp::NodeOptions &options);

    void error_callback(const amr_v4_msgs_srvs::msg::Error::SharedPtr msg);
    void battery_callback(const amr_v4_msgs_srvs::msg::Battery::SharedPtr msg);
    void motor_callback(const amr_v4_msgs_srvs::msg::Motor::SharedPtr msg);
    void robot_callback(const amr_v4_msgs_srvs::msg::Robot::SharedPtr msg);
    void estop_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void station_callback(const std_msgs::msg::String::SharedPtr msg);  // Station callback
    void mode_callback(const bool msg);
    void pin_callback(const bool msg);
    
    void createChargingPublisher(); // Dynamically create the charging publisher
    void destroyChargingPublisher(); // Dynamically destroy the charging publisher
    void publishChargingMessage(bool start_charging_main); // Publish the charging message
    void updateChargingState(bool charging_active);
    
    std::shared_ptr<rclcpp::Node> get_raw_node() override;
    std::string get_node_name() const override;
    std::map<std::string, std::vector<std::string>> get_topic_names_and_types() const override;
    void setSelfPtr(std::shared_ptr<AmrNode> self);
    
    std::string getRobotId();  
    void retrieveStationIdFromMongoDB(); //MongoDB station id
    bool slam_lidar;
    bool estop_lidar;
    bool camera;

    float voltage;
    QString temperature;
    float current;
    float soc;

    QString output_current_right;
    QString error_right;
    QString output_current_left;
    QString error_left;
    QString output_current_pin;
    QString error_pin;

    QString status;
    QString station_id;  // Holds the station ID as a QString
    QString mission_type;
    
    bool estop;

signals:
    void changedError(const bool slam_lidar,
                      const bool estop_lidar,
                      const bool camera);

    void changedBattery(const float voltage,
                        const float current,
                        const float soc,
                        const QString temperature);

    void changedMotor(const QString &output_current_right,
                      const QString &output_current_left,
                      const QString &output_current_pin,
                      const QString &error_right,
                      const QString &error_left,
                      const QString &error_pin);
    void changedRobot(const QString &data);
    void changedCycles(const QString &totalCycles);
    void changedEstop(const bool &data);
    void changedStation(const QString &station_id);  // Signal for station ID changes

private:
    std::shared_ptr<AmrNode> self_node_ptr_;
    rclcpp::Subscription<amr_v4_msgs_srvs::msg::Error>::SharedPtr error_sub_;
    rclcpp::Subscription<amr_v4_msgs_srvs::msg::Battery>::SharedPtr battery_sub_;
    rclcpp::Subscription<amr_v4_msgs_srvs::msg::Motor>::SharedPtr motor_sub_;
    rclcpp::Subscription<amr_v4_msgs_srvs::msg::Robot>::SharedPtr robot_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_sub_;  // Station subscription
    rclcpp::Publisher<amr_v4_msgs_srvs::msg::Mode>::SharedPtr mode_pub_;
    rclcpp::Publisher<amr_v4_msgs_srvs::msg::Pin>::SharedPtr pin_pub_;


    std::string robot_name_;
    rclcpp::Publisher<amr_v4_msgs_srvs::msg::Charging>::SharedPtr charging_pub_;


};

#endif // BATTERY_NODE_H