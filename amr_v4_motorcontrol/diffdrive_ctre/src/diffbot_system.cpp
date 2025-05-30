#include "diffdrive_ctre/diffbot_system.hpp"

namespace diffdrive_ctre
{

void DiffDriveCTREHardware::errorloop()
{
  mr_.flt_error = false;
  
  if (ml_.flt_error == true)
  { //to add action
    RCLCPP_INFO(
      rclcpp::get_logger("Motor Hardware"), "Got Error from MCU, please see screen or msg type in ROS");
  }
}

void DiffDriveCTREHardware::set_leds()
{
  static LEDState last_state = LEDState::OFF;
  
  ctre::phoenix::led::LarsonAnimation larson_animation(255, 165, 0, 0, 0.01, 8, ctre::phoenix::led::LarsonAnimation::BounceMode::Front, 1, 0);
  ctre::phoenix::led::RainbowAnimation rainbow_animation(0.5, 0.01, 8, false, 0);
  ctre::phoenix::led::FireAnimation fire_animation(0.5, 0.01, 8, 0.5, 0.5, false, 0);
  ctre::phoenix::led::SingleFadeAnimation breathing_white(255, 255, 255, 0.5, 0.02, 8, 0);

  auto clear_all_animations = [this]() {
    candle_fr.ClearAnimation(0);
    candle_fl.ClearAnimation(0);
    candle_sr.ClearAnimation(0);
    candle_sl.ClearAnimation(0);
  };

  LEDState new_state = LEDState::OFF; 

  if (latest_mode_cmd.mode == 0) 
  {
    if (latest_light_cmd.charging == 1) 
    {
      new_state = LEDState::CHARGING;
    } 
    else 
    {
      new_state = LEDState::MANUAL;
    }
  } 
  else if (latest_mode_cmd.mode == 1) 
  {
    if (latest_light_cmd.charging == 1 && cfg_.estop == 0) 
    {
      new_state = LEDState::CHARGING;
    } 
    else if (cfg_.estop == 1 && latest_light_cmd.charging == 1) 
    {
      new_state = LEDState::ESTOPCHARGE;
    }
    else if (cfg_.estop == 1 && latest_light_cmd.charging == 0) 
    {
      new_state = LEDState::ESTOP;
    } 
    else if (mr_.duty_right > 0.03 && ml_.duty_left > 0.03 && abs(nav_twist_cmd_) == 0.00 && latest_mode_cmd.line_follow == 0)
    {
      new_state = LEDState::MOVING_FORWARD;
    } 
    else if (nav_twist_cmd_ < -0.03 && latest_mode_cmd.line_follow == 0) 
    {
      new_state = LEDState::MOVING_RIGHT;
    } 
    else if (nav_twist_cmd_ > 0.03 && latest_mode_cmd.line_follow == 0) 
    {
      new_state = LEDState::MOVING_LEFT;
    } 
    else if (latest_mode_cmd.inter_stop == 1 && mr_.duty_right == 0.00 && ml_.duty_left == 0.00) 
    {
      new_state = LEDState::INTER_STOP;
    } 
    else if (latest_light_cmd.unload == 1 && cfg_.estop == 0)
    {
      new_state = LEDState::UNLOADING;
    }
    else if (latest_light_cmd.forklift == 1 && cfg_.estop == 0)
    {
      new_state = LEDState::FORKLIFT;
    }
    else if (latest_light_cmd.home == 1 && cfg_.estop == 0)
    {
      new_state = LEDState::HOME;
    }
    else if (latest_mode_cmd.line_follow == 1 && latest_light_cmd.forklift == 0 && latest_light_cmd.unload == 0)
    {
      new_state = LEDState::LF;
    }
    else 
    {
      new_state = LEDState::AUTO;
    }
  }

  
  if (new_state != last_state) 
  {
    clear_all_animations(); 
    last_state = new_state; 

    switch (new_state) 
    {
        case LEDState::CHARGING:
            // Front LEDs (FR, FL) - Yellow (255, 255, 0) indicating charging state
            candle_fr.SetLEDs(255, 255, 255);
            candle_fl.SetLEDs(255, 255, 255);
            // Side LEDs (SR, SL) - Blue (0, 0, 255) indicating standby or charging mode
            candle_sr.SetLEDs(0, 0, 255);
            candle_sl.SetLEDs(0, 0, 255);
            break;

        case LEDState::ESTOPCHARGE:
            // Front LEDs (FR, FL) - Yellow (255, 255, 0) indicating charging state
            candle_fr.SetLEDs(255, 0, 0);
            candle_fl.SetLEDs(255, 0, 0);
            // Side LEDs (SR, SL) - Blue (0, 0, 255) indicating standby or charging mode
            candle_sr.SetLEDs(255, 255, 0);
            candle_sl.SetLEDs(255, 255, 0);
            break;
    
        case LEDState::MANUAL:
            // All LEDs (FR, FL, SR, SL) - Yellow (255, 255, 0) indicating manual control mode
            candle_fr.SetLEDs(255, 140, 0);
            candle_fl.SetLEDs(255, 140, 0);
            candle_sr.SetLEDs(255, 140, 0);
            candle_sl.SetLEDs(255, 140, 0);
            break;
    
        case LEDState::MOVING_FORWARD:
            // Front LEDs (FR, FL) - White (255, 255, 255) indicating movement forward
            candle_fr.SetLEDs(255, 255, 255);
            candle_fl.SetLEDs(255, 255, 255);
            // Side LEDs (SR, SL) - Green (0, 255, 0) indicating safe forward motion
            candle_sr.SetLEDs(0, 255, 0);
            candle_sl.SetLEDs(0, 255, 0);
            break;
    
        case LEDState::MOVING_RIGHT:
            // Front Right & Side Right LEDs (FR, SR) - Animated (Larson scanner effect) for right movement indication
            candle_fr.Animate(larson_animation);
            candle_sr.Animate(larson_animation);
            // Front Left LED (FL) - White (255, 255, 255) indicating normal state
            candle_fl.SetLEDs(255, 255, 255);
            // Side Left LED (SL) - Green (0, 255, 0) indicating safe zone
            candle_sl.SetLEDs(0, 255, 0);
            break;
    
        case LEDState::MOVING_LEFT:
            // Front Left & Side Left LEDs (FL, SL) - Animated (Larson scanner effect) for left movement indication
            candle_fl.Animate(larson_animation);
            candle_sl.Animate(larson_animation);
            // Front Right LED (FR) - White (255, 255, 255) indicating normal state
            candle_fr.SetLEDs(255, 255, 255);
            // Side Right LED (SR) - Green (0, 255, 0) indicating safe zone
            candle_sr.SetLEDs(0, 255, 0);
            break;
    
        case LEDState::INTER_STOP:
            // All LEDs (FR, FL, SR, SL) - Fire animation indicating intermediate stop
            candle_fr.Animate(fire_animation);
            candle_fl.Animate(fire_animation);
            candle_sr.Animate(fire_animation);
            candle_sl.Animate(fire_animation);
            break;
    
        case LEDState::ESTOP:
            // All LEDs (FR, FL, SR, SL) - Red (255, 0, 0) indicating emergency stop (E-STOP)
            candle_fr.SetLEDs(255, 0, 0);
            candle_fl.SetLEDs(255, 0, 0);
            candle_sr.SetLEDs(255, 0, 0);
            candle_sl.SetLEDs(255, 0, 0);
            break;
    
        case LEDState::AUTO:
            // Front LEDs (FR, FL) - White (255, 255, 255) indicating automatic mode
            candle_fr.SetLEDs(255, 255, 255);
            candle_fl.SetLEDs(255, 255, 255);
            // Side LEDs (SR, SL) - Green (0, 255, 0) indicating safe movement
            candle_sr.SetLEDs(0, 255, 0);
            candle_sl.SetLEDs(0, 255, 0);
            break;

        case LEDState::HOME:
            // Front LEDs (FR, FL) - White (255, 255, 255) indicating automatic mode
            candle_fr.Animate(breathing_white);
            candle_fl.Animate(breathing_white);
            // Side LEDs (SR, SL) - Green (0, 255, 0) indicating safe movement
            candle_sr.Animate(breathing_white);
            candle_sl.Animate(breathing_white);
            break;

        case LEDState::FORKLIFT:
    
            candle_fr.SetLEDs(50, 50, 50);
            candle_fl.SetLEDs(50, 50, 50);
            
            candle_sr.SetLEDs(50, 50, 50);
            candle_sl.SetLEDs(50, 50, 50);
            break;
        
        case LEDState::UNLOADING:
            
            candle_fr.SetLEDs(50, 50, 50);
            candle_fl.SetLEDs(50, 50, 50);
            
            candle_sr.SetLEDs(50, 50, 50);
            candle_sl.SetLEDs(50, 50, 50);
            break;
        
        case LEDState::LF:
            
            candle_fr.SetLEDs(255, 255, 255);
            candle_fl.SetLEDs(255, 255, 255);
            
            candle_sr.SetLEDs(255, 255, 255);
            candle_sl.SetLEDs(255, 255, 255);
            break;
    
        default:
            // All LEDs (FR, FL, SR, SL) - Off (0, 0, 0) for undefined state
            candle_fr.SetLEDs(0, 0, 0);
            candle_fl.SetLEDs(0, 0, 0);
            candle_sr.SetLEDs(0, 0, 0);
            candle_sl.SetLEDs(0, 0, 0);
            break;
    }
    
  }
}


hardware_interface::CallbackReturn DiffDriveCTREHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  const char* env_var = "ROBOT_MODEL";
  const char* var_value_cstr = std::getenv(env_var);
  std::cout << "Starting Program for: " << var_value_cstr << std::endl;

  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS && var_value_cstr != nullptr)
  {
    std::cout << "Please check environemnt variable ROBOT_MODEL " << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string robot_name(var_value_cstr);
  hw_start_sec_ = stod(info_.hardware_parameters["start_delay"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["stop_delay"]);
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  cfg_.inter = info_.hardware_parameters["interface"];
  cfg_.P = stod(info_.hardware_parameters["P"]);
  cfg_.I = stod(info_.hardware_parameters["I"]);
  cfg_.D = stod(info_.hardware_parameters["D"]);
  cfg_.V = stod(info_.hardware_parameters["V"]);
  cfg_.G = stod(info_.hardware_parameters["G"]);
  cfg_.S = stod(info_.hardware_parameters["S"]);
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_ticks"]);
  cfg_.sensor_mul = stod(info_.hardware_parameters["sensor_mul"]);
  cfg_.loop_rate = std::stoi(info_.hardware_parameters["loop_rate"]);
  cfg_.rads_rots = stod(info_.hardware_parameters["rads_rots_conv"]);
  cfg_.gear_ratio = stod(info_.hardware_parameters["gear_ratio"]);
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__ns:=/"+ robot_name, "-r", "__node:=Motor_Status"});
  node_ = rclcpp::Node::make_shared("_",options);
  pub_ = node_->create_publisher<amr_v4_msgs_srvs::msg::Motor>("talonFX_motors/status", 10);
  estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("e_stop", 10);
  lights_sub_ = node_->create_subscription<amr_v4_msgs_srvs::msg::Lights>("lights", 10,
    [this](const amr_v4_msgs_srvs::msg::Lights::SharedPtr light_cmd) { latest_light_cmd = *light_cmd; });
  estop_mode_sub_ =  node_->create_subscription<amr_v4_msgs_srvs::msg::Mode>("mode", 10,
    [this](const amr_v4_msgs_srvs::msg::Mode::SharedPtr mode_cmd) { latest_mode_cmd = *mode_cmd;});
  
  vel_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("diffbot_base_controller/cmd_vel_unstamped", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr vel_cmd) {
      nav_twist_cmd_ = vel_cmd->angular.z;
    });

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCTREHardware"),
        "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCTREHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCTREHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCTREHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCTREHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveCTREHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
 //left wheel state
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.left_wheel_name, hardware_interface::HW_IF_POSITION, &ml_.motor_left_pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &ml_.motor_left_vel_fdb));

  //right wheel state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.right_wheel_name, hardware_interface::HW_IF_POSITION, &mr_.motor_right_pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.right_wheel_name, hardware_interface::HW_IF_VELOCITY, &mr_.motor_right_vel_fdb));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveCTREHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    cfg_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &ml_.motor_left_cmd));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    cfg_.right_wheel_name, hardware_interface::HW_IF_VELOCITY, &mr_.motor_right_cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveCTREHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Activating ...please wait...");

    ctre::phoenix::StatusCode Status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    ctre::phoenix::StatusCode Status_1 = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    ctre::phoenix::StatusCode Status_2 = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    //gpio_.reset = 0;
    gpio_.init();
    mp_.low_sof_limit = 0.99;
    mp_.high_sof_limit = -0.32;

    //motor 1 controller
    motor_right.ClearStickyFaults();
    motor_right.GetPosition().SetUpdateFrequency(cfg_.loop_rate*1_Hz);
    talon_right_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    talon_right_config.Slot0.kP = cfg_.P;
    talon_right_config.Slot0.kI = cfg_.I;
    talon_right_config.Slot0.kD = cfg_.D;
    talon_right_config.Slot0.kV = cfg_.V;
    talon_right_config.Slot0.kG = cfg_.G;
    talon_right_config.Slot0.kS = cfg_.S;
    talon_right_config.Audio.BeepOnBoot = true;
    talon_right_config.Audio.BeepOnConfig = true;
    talon_right_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon_right_config.CurrentLimits.SupplyCurrentThreshold = 4.5;
    talon_right_config.CurrentLimits.SupplyTimeThreshold = 2.0;
    talon_right_config.CurrentLimits.SupplyCurrentLimit = 2.5;
    talon_right_config.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    talon_right_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    talon_right_config.Feedback.SensorToMechanismRatio = cfg_.gear_ratio;
    right_motor_control.Slot = 0;
    right_motor_control.UpdateFreqHz = cfg_.loop_rate*1_Hz;
    motor_right.GetStatorCurrent().SetUpdateFrequency(cfg_.loop_rate*1_Hz);
    motor_right.GetStickyFault_Hardware().SetUpdateFrequency(cfg_.loop_rate*1_Hz);

    // //motor 2 controller
    motor_left.ClearStickyFaults();
    motor_left.GetPosition().SetUpdateFrequency(cfg_.loop_rate*1_Hz);
    talon_left_config.Slot0.kP = cfg_.P;
    talon_left_config.Slot0.kI = cfg_.I;
    talon_left_config.Slot0.kD = cfg_.D;
    talon_left_config.Slot0.kV = cfg_.V;
    talon_left_config.Slot0.kG = cfg_.G;
    talon_left_config.Slot0.kS = cfg_.S;
    talon_left_config.Audio.BeepOnBoot = true;
    talon_left_config.Audio.BeepOnConfig = true;
    talon_left_config.MotorOutput.Inverted = phoenix6::signals::InvertedValue::Clockwise_Positive;
    talon_left_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon_left_config.CurrentLimits.SupplyCurrentThreshold = 4.5;
    talon_left_config.CurrentLimits.SupplyTimeThreshold = 2.0;
    talon_left_config.CurrentLimits.SupplyCurrentLimit = 2.5;
    talon_left_config.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    talon_left_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    talon_left_config.Feedback.SensorToMechanismRatio = cfg_.gear_ratio;
    left_motor_control.Slot = 0;
    left_motor_control.UpdateFreqHz = cfg_.loop_rate*1_Hz;
    motor_left.GetStatorCurrent().SetUpdateFrequency(cfg_.loop_rate*1_Hz);
    motor_left.GetStickyFault_Hardware().SetUpdateFrequency(cfg_.loop_rate*1_Hz);

    // //pin controller
    pin_motor.ClearStickyFaults();
    pin_motor.GetPosition().SetUpdateFrequency(cfg_.loop_rate*1_Hz);
    talon_pin_config.Slot0.kP = 0.1;
    talon_pin_config.Slot0.kI = 0.1;
    talon_pin_config.Slot0.kD = 0.0;
    talon_pin_config.Slot0.kV = 0.02;
    talon_pin_config.Slot0.kS = 0.01;
    talon_pin_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon_pin_config.CurrentLimits.SupplyCurrentThreshold = 5.0;
    talon_pin_config.CurrentLimits.SupplyTimeThreshold = 3.0;
    talon_pin_config.CurrentLimits.SupplyCurrentLimit = 2.5;
    
    talon_pin_config.Audio.BeepOnBoot = true;
    talon_pin_config.Audio.BeepOnConfig = true;
    talon_pin_config.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    talon_pin_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    talon_pin_config.MotionMagic.MotionMagicAcceleration = 4;
    talon_pin_config.MotionMagic.MotionMagicCruiseVelocity = 5;
    talon_pin_config.MotionMagic.MotionMagicJerk = 1600;
    talon_pin_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    talon_pin_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    talon_pin_config.HardwareLimitSwitch.ForwardLimitEnable = false;
    talon_pin_config.HardwareLimitSwitch.ReverseLimitEnable = false;
    //talon_pin_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mp_.high_sof_limit;
    //talon_pin_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mp_.low_sof_limit;
    pin_motor_control.Slot = 0;
    pin_motor_control.UpdateFreqHz = cfg_.loop_rate*1_Hz;
    pin_motor.GetStatorCurrent().SetUpdateFrequency(cfg_.loop_rate*1_Hz);
    pin_motor.GetStickyFault_Hardware().SetUpdateFrequency(cfg_.loop_rate*1_Hz);

    //candle config
    candle_fr.ClearStickyFaults();
    candle_config.stripType = ctre::phoenix::led::LEDStripType::RGB;
    candle_config.brightnessScalar = 1.0;
    candle_config.disableWhenLOS = true;
    candle_config.statusLedOffWhenActive = true;
    candle_config.v5Enabled = false;
    candle_fr.ConfigAllSettings(candle_config,1000);

    candle_sr.ClearStickyFaults();
    candle_config.stripType = ctre::phoenix::led::LEDStripType::RGB;
    candle_config.brightnessScalar = 1.0;
    candle_config.disableWhenLOS = true;
    candle_config.statusLedOffWhenActive = true;
    candle_config.v5Enabled = false;
    candle_sr.ConfigAllSettings(candle_config,1000);

    candle_fl.ClearStickyFaults();
    candle_config.stripType = ctre::phoenix::led::LEDStripType::RGB;
    candle_config.brightnessScalar = 1.0;
    candle_config.disableWhenLOS = true;
    candle_config.statusLedOffWhenActive = true;
    candle_config.v5Enabled = false;
    candle_fl.ConfigAllSettings(candle_config,1000);

    candle_sl.ClearStickyFaults();
    candle_config.stripType = ctre::phoenix::led::LEDStripType::RGB;
    candle_config.brightnessScalar = 1.0;
    candle_config.disableWhenLOS = true;
    candle_config.statusLedOffWhenActive = true;
    candle_config.v5Enabled = false;
    candle_fr.ConfigAllSettings(candle_config,1000);
        
    for (int i = 0; i < 10; ++i) {
        rclcpp::sleep_for(std::chrono::seconds(1));
        Status = motor_right.GetConfigurator().Apply(talon_right_config,1_s);
        Status_1 = pin_motor.GetConfigurator().Apply(talon_pin_config,1_s);
        Status_2 = motor_left.GetConfigurator().Apply(talon_left_config,1_s);
       

        if (Status.IsOK()  && Status_2.IsOK() && Status_1.IsOK()) {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "All 3 motors CAN is good !");
            break;
        } else if (!Status.IsOK()) {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Motor Right gave error on start !");
        } else if (!Status_1.IsOK()) {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Motor Pin gave error on start !");
        } else if (!Status_2.IsOK()) {
            RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Motor Left gave error on start !");
        }
    }

    //common motor config
    ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(motor_left, motor_right, pin_motor);

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveCTREHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Deactivating ...please wait...");
    talon_right_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    talon_left_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    talon_pin_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    motor_right.GetConfigurator().Apply(talon_right_config);
    motor_left.GetConfigurator().Apply(talon_right_config);
    pin_motor.GetConfigurator().Apply(talon_right_config);
    motor_right.SetControl(ctre::phoenix6::controls::NeutralOut{});
    motor_left.SetControl(ctre::phoenix6::controls::NeutralOut{});
    pin_motor.SetControl(ctre::phoenix6::controls::NeutralOut{});
    candle_fr.ClearAnimation(0);
    candle_fl.ClearAnimation(0);
    candle_sr.ClearAnimation(0);
    candle_sl.ClearAnimation(0);
    candle_fr.SetLEDs(0, 0, 0);
    candle_fl.SetLEDs(0, 0, 0);
    candle_sr.SetLEDs(0, 0, 0);
    candle_sl.SetLEDs(0, 0, 0);
    ctre::phoenix::unmanaged::FeedEnable(0);
    gpio_.cleanup();
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveCTREHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  //RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Estop status: %s", gpio_.estop ? "true" : "false");
  //RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Estop status SW: %s", latest_mode_cmd.sw_estop ? "true" : "false");
  //RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "Estop status Inter: %s", latest_mode_cmd.inter_stop ? "true" : "false");
  size_t num_publishers = node_->count_publishers("mode");
 //&& latest_mode_cmd.line_follow == 0
  gpio_.readgpio();

  if ((gpio_.estop == 1 && latest_mode_cmd.mode == 1 && num_publishers > 0) || (latest_mode_cmd.inter_stop == 1 && num_publishers > 0))
  {
     //cfg_.estop = gpio_.estop;
     cfg_.estop = 1;
  } else {
     cfg_.estop = 0;
  }

  mr_.motor_right_pos = motor_right.GetPosition().GetValueAsDouble()*(2*M_PI);
  mr_.motor_right_vel_fdb = motor_right.GetVelocity().GetValueAsDouble()*(2*M_PI);
  ml_.motor_left_pos = motor_left.GetPosition().GetValueAsDouble()*(2*M_PI);
  ml_.motor_left_vel_fdb = motor_left.GetVelocity().GetValueAsDouble()*(2*M_PI);
  mp_.feeback = pin_motor.GetPosition().GetValueAsDouble();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_ctre::DiffDriveCTREHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  errorloop();
  set_leds();
  //gpio_.readgpio();
  
  if ((cfg_.estop == 0) && (latest_mode_cmd.inter_stop == 0)) {
    ctre::phoenix::unmanaged::FeedEnable(50);
    mr_.duty_right = mr_.motor_right_cmd/(2*M_PI);
    ml_.duty_left = ml_.motor_left_cmd/(2*M_PI);
    motor_right.SetControl(right_motor_control.WithVelocity(mr_.duty_right*1_tps));
    motor_left.SetControl(left_motor_control.WithVelocity(ml_.duty_left*1_tps));

    //gpio_.reset = 0;
  } else if ((cfg_.estop == 1)) {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveCTREHardware"), "ESTOP Commanded!");
    ctre::phoenix::unmanaged::FeedEnable(0);
    motor_right.SetControl(ctre::phoenix6::controls::NeutralOut{});
    motor_left.SetControl(ctre::phoenix6::controls::NeutralOut{});
    //gpio_.reset = 1;
  }
    // //motor1
  auto message = amr_v4_msgs_srvs::msg::Motor();
  message.output_current_right = motor_right.GetStatorCurrent().ToString();
  message.error_right = motor_right.GetStickyFault_Hardware().ToString();

  // //motor2
  message.output_current_left = motor_left.GetStatorCurrent().ToString();
  message.error_left = motor_left.GetStickyFault_Hardware().ToString();

    // //motor pin
  message.output_current_pin = pin_motor.GetStatorCurrent().ToString();
  message.error_pin = pin_motor.GetStickyFault_Hardware().ToString();
  message.target_pos_high = mp_.high_sof_limit;
  message.target_pos_low = mp_.low_sof_limit;
  message.current_pos = mp_.feeback;

  //candles
  message.candle_0_fault = candle_fr.GetFaults(candle_faults);
  message.candle_0_current = candle_fr.GetCurrent();
  message.candle_1_fault = candle_sr.GetFaults(candle_faults);
  message.candle_1_current = candle_sr.GetCurrent();
  message.candle_2_fault = candle_fl.GetFaults(candle_faults);
  message.candle_2_current = candle_fl.GetCurrent();
  message.candle_3_fault = candle_sl.GetFaults(candle_faults);
  message.candle_3_current = candle_sl.GetCurrent();


  //estop
  auto message_estop = std_msgs::msg::Bool();
  message_estop.data = bool(cfg_.estop);
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
    estop_pub_->publish(message_estop);
    pub_->publish(message);
  }  
  
  return hardware_interface::return_type::OK;
}
}  
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_ctre::DiffDriveCTREHardware, hardware_interface::SystemInterface)