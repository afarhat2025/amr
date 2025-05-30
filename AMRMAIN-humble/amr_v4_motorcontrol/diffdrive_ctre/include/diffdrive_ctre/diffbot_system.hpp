#ifndef DIFFDRIVE_CTRE__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_CTRE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdlib>

#include <cmath>
#include <chrono>
#include <limits>
#include <cstddef>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diffdrive_ctre/visibility_control.h"

#include "rclcpp/rclcpp.hpp"
//#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

//transmission
//#include "transmission_interface/transmission.hpp"

// CTRE
// #include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>
//#include <ctre/phoenix6/CANBus.hpp>
// #include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

//msg srv publisher
#include "amr_v4_msgs_srvs/msg/motor.hpp"
#include "amr_v4_msgs_srvs/msg/robot.hpp"
#include "amr_v4_msgs_srvs/msg/mode.hpp"
#include "amr_v4_msgs_srvs/msg/lights.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

//gpio
#include "diffdrive_ctre/gpio_comms_motor.hpp"

//units
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>

//joystick
#include <sensor_msgs/msg/joy.hpp>

#include <iostream>
#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.hpp"
#include "ctre/phoenix/led/ColorFlowAnimation.h"
#include "ctre/phoenix/led/FireAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/RgbFadeAnimation.h"
#include "ctre/phoenix/led/SingleFadeAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"
#include "ctre/phoenix/led/TwinkleAnimation.h"
#include "ctre/phoenix/led/TwinkleOffAnimation.h"

namespace diffdrive_ctre
{

class DiffDriveCTREHardware : public hardware_interface::SystemInterface
{
  ctre::phoenix6::hardware::TalonFX motor_right{2,"slowbro"};
  ctre::phoenix6::hardware::TalonFX motor_left{1,"slowbro"};
  ctre::phoenix6::hardware::TalonFX pin_motor{3,"slowbro"};

  ctre::phoenix6::configs::TalonFXConfiguration talon_right_config{};
  ctre::phoenix6::configs::TalonFXConfiguration talon_left_config{};
  phoenix6::configs::TalonFXConfiguration talon_pin_config{};

  ctre::phoenix6::controls::VelocityDutyCycle right_motor_control{0_tps};
  ctre::phoenix6::controls::VelocityDutyCycle left_motor_control{0_tps};
  ctre::phoenix6::controls::MotionMagicDutyCycle pin_motor_control{0_tr};

  ctre::phoenix6::controls::NeutralOut common_config{};

  ctre::phoenix::led::CANdle candle_fr{4,"slowbro"};
  
  ctre::phoenix::led::CANdleConfiguration candle_config{};
  ctre::phoenix::led::CANdleFaults candle_faults{};

  ctre::phoenix::led::CANdle candle_sr{5,"slowbro"};

  ctre::phoenix::led::CANdle candle_fl{6,"slowbro"};

  ctre::phoenix::led::CANdle candle_sl{0,"slowbro"};

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  std::string inter = "";
  int enc_counts_per_rev = 0;
  double P = 0;
  double I = 0;
  double D = 0;
  double V = 0;
  double S = 0;
  double G = 0;
  double sensor_mul = 0;
  double rads_rots = 0;
  double gear_ratio = 0;
  int loop_rate = 0;
  int estop = 0;
  int prox = 0;

};

enum class LEDState
{
    OFF,
    CHARGING,
    MANUAL,
    AUTO,
    MOVING_FORWARD,
    MOVING_LEFT,
    MOVING_RIGHT,
    UNLOADING,
    INTER_STOP,
    ESTOP,
    ESTOPCHARGE,
    HOME,
    LF,
    FORKLIFT
};

struct motor_right_talon
{
  double duty_right = 0;
  double bus_vltg = 0;
  double bus_temp = 0;
  double motor_right_vel_fdb = 0;
  double motor_right_pos = 0;
  double motor_right_cmd = 0;
  bool flt_error = false;

};

struct motor_left_talon
{
  double duty_left = 0;
  double motor_left_vel_fdb = 0;
  double motor_left_pos = 0;
  double motor_left_cmd = 0;
  bool flt_error = false;
  
};

struct motor_pin_talon
{
  double feeback = 0;
  double pos_cmd = 0;
  double high_sof_limit = 0;
  double low_sof_limit = 0;
  bool flt_error = false;
  int pin = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveCTREHardware);

  DIFFDRIVE_CTRE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  DIFFDRIVE_CTRE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFDRIVE_CTRE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFDRIVE_CTRE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_CTRE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_CTRE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFFDRIVE_CTRE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  Config cfg_;
  motor_right_talon mr_;
  motor_left_talon ml_;
  motor_pin_talon mp_;
  double hw_start_sec_;
  double hw_stop_sec_;
  gpio_comms gpio_;
  double nav_twist_cmd_ = 0.0;
  LEDState current_led_state = LEDState::MANUAL;
  
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  size_t num_publishers = 0;
  
  void errorloop();
  void set_leds();
  void clear_all_animations();

  rclcpp::Publisher<amr_v4_msgs_srvs::msg::Motor>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<amr_v4_msgs_srvs::msg::Mode>::SharedPtr estop_mode_sub_;
  rclcpp::Subscription<amr_v4_msgs_srvs::msg::Lights>::SharedPtr lights_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_sub_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;
  amr_v4_msgs_srvs::msg::Mode latest_mode_cmd;
  amr_v4_msgs_srvs::msg::Lights latest_light_cmd;


};

}  // namespace diffdrive_ctre

#endif  // DIFFDRIVE_CTRE__DIFFBOT_SYSTEM_HPP_