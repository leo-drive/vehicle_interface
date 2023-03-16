// Copyright 2023 Leo Drive Teknoloji A.Ş.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <leo_vcu_driver/leo_vcu_driver.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
using namespace std::placeholders;

using rcl_interfaces::msg::ParameterDescriptor;
using rclcpp::ParameterValue;
using namespace std::chrono_literals;

LeoVcuDriver::LeoVcuDriver()
: Node("leo_vcu_driver"),
  plugin_loader_("leo_vcu_driver", "leo_vcu_driver::LeoVcuDriverPlugin"),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()),
  updater_(this)
{
  /* Parameters */
  interface_mod_ = declare_parameter("interface_mod", "CAN");
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000.0);
  data_send_rate_ = declare_parameter("data_send_rate", 100.0);
  /* parameters for vehicle specifications */
  wheel_base_ = static_cast<float>(vehicle_info_.wheel_base_m);
  reverse_gear_enabled_ = declare_parameter("reverse_gear_enabled", true);
  gear_shift_velocity_threshold =
    static_cast<float>(declare_parameter("gear_shift_velocity_threshold", 0.1));
  max_steering_wheel_angle =
    static_cast<float>(declare_parameter("max_steering_wheel_angle", 750.0));
  min_steering_wheel_angle =
    static_cast<float>(declare_parameter("min_steering_wheel_angle", -750.0));
  max_steering_wheel_angle_rate =
    static_cast<float>(declare_parameter("max_steering_wheel_angle_rate", 300.0));
  check_steering_angle_rate = declare_parameter("check_steering_angle_rate", true);
  enable_emergency = declare_parameter("enable_emergency", true);
  enable_cmd_timeout_emergency = declare_parameter("enable_cmd_timeout_emergency", true);
  steering_offset = static_cast<float>(declare_parameter("steering_offset", 0.0));
  emergency_stop_acceleration =
    static_cast<float>(declare_parameter("emergency_stop_acceleration", -5.0));
  soft_stop_acceleration = static_cast<float>(declare_parameter("soft_stop_acceleration", -1.5));
  add_emergency_acceleration_per_second =
    static_cast<float>(declare_parameter("add_emergency_acceleration_per_second", -0.5));
  enable_long_actuation_mode = declare_parameter("enable_long_actuation_mode", false);

  /* Subscribers */

  // From Autoware
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1, std::bind(&LeoVcuDriver::ctrl_cmd_callback, this, _1));
  gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&LeoVcuDriver::gear_cmd_callback, this, _1));
  turn_indicators_cmd_sub_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
      std::bind(&LeoVcuDriver::turn_indicators_cmd_callback, this, _1));
  hazard_lights_cmd_sub_ =
    create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
      std::bind(&LeoVcuDriver::hazard_lights_cmd_callback, this, _1));
  engage_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    "/autoware/engage", rclcpp::QoS{1}, std::bind(&LeoVcuDriver::engage_cmd_callback, this, _1));
  gate_mode_sub_ = create_subscription<tier4_control_msgs::msg::GateMode>(
    "/control/current_gate_mode", 1, std::bind(&LeoVcuDriver::gate_mode_cmd_callback, this, _1));
  emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&LeoVcuDriver::emergency_cmd_callback, this, _1));
  emergency_state_sub_ = this->create_subscription<autoware_auto_system_msgs::msg::EmergencyState>(
    "/system/emergency/emergency_state", 1, std::bind(&LeoVcuDriver::onEmergencyState, this, _1));
  sub_hazard_status_stamped_ =
    create_subscription<autoware_auto_system_msgs::msg::HazardStatusStamped>(
      "/system/emergency/hazard_status", rclcpp::QoS{1},
      std::bind(&LeoVcuDriver::onHazardStatusStamped, this, _1));
  autoware_state_sub_ = create_subscription<autoware_auto_system_msgs::msg::AutowareState>(
    "/autoware/state", rclcpp::QoS(1), std::bind(&LeoVcuDriver::onAutowareState, this, _1));
  actuation_cmd_sub_ = create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
    "/control/command/actuation_cmd", rclcpp::QoS(1),
    std::bind(&LeoVcuDriver::actuator_cmd_callback, this, _1));

  /* Publishers */

  // To Autoware
  control_mode_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  vehicle_twist_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  gear_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  turn_indicators_status_pub_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  hazard_lights_status_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  steering_wheel_status_pub_ =
    create_publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>(
      "/vehicle/status/steering_wheel_status", 1);
  llc_error_pub_ = create_publisher<std_msgs::msg::String>("/interface/status/llc_status", 1);
  hand_brake_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::HandBrakeReport>(
          "/vehicle/status/hand_brake", rclcpp::QoS{1});
  headlights_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::HeadlightsReport>(
          "/vehicle/status/headlights", rclcpp::QoS{1});
  vehicle_state_report_pub_ = create_publisher<leo_vcu_msgs::msg::StateReport>(
    "/vehicle/status/status_report", rclcpp::QoS{1});

  // System error diagnostic
  updater_.setHardwareID("vehicle_error_monitor");
  updater_.add("motor_running_error", this, &LeoVcuDriver::checkMotorRunningError);
  updater_.add("kl75_error", this, &LeoVcuDriver::checkKl75Error);
  updater_.add("pds_timeout_error", this, &LeoVcuDriver::checkPDSTimeoutError);
  updater_.add("pds_bus_error", this, &LeoVcuDriver::checkPDSBusError);
  updater_.add("by_wire_power_error", this, &LeoVcuDriver::checkByWireError);
  updater_.add("epas_power_error", this, &LeoVcuDriver::checkEPASPowerError);
  updater_.add("brake_power_error", this, &LeoVcuDriver::checkBrakePowerError);
  updater_.add("throttle_ecu_timeout_error", this, &LeoVcuDriver::checkThrottleTimeoutError);
  updater_.add("g29_timeout_error", this, &LeoVcuDriver::checkG29TimeoutError);
  updater_.add("epas_system_error", this, &LeoVcuDriver::checkEPASSystemError);
  updater_.add("epas_timeout_error", this, &LeoVcuDriver::checkEPASTimeoutError);
  updater_.add("brake_system_error", this, &LeoVcuDriver::checkBrakeSystemError);
  updater_.add("brake_timeout_error", this, &LeoVcuDriver::checkBrakeTimeoutError);
  updater_.add("pc_timeout_error", this, &LeoVcuDriver::checkPCTimeoutError);

  // Load interface plugin
  if (interface_mod_ == "CAN"){
    driver_interface_plugin_ =
      plugin_loader_.createSharedInstance("leo_vcu_driver::can_interface::CanInterface");
    driver_interface_plugin_->initialize(this);
  } else { // SERIAL
    driver_interface_plugin_ =
      plugin_loader_.createSharedInstance("leo_vcu_driver::serial_interface::SerialInterface");
    driver_interface_plugin_->initialize(this);
  }

  // Timer
  const auto period_ns = rclcpp::Rate(data_send_rate_).period();
  tim_data_sender_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&LeoVcuDriver::llc_interface_adapter, this));

}

void LeoVcuDriver::onHazardStatusStamped(
  const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  hazard_status_stamped_ = msg;
}

void LeoVcuDriver::ctrl_cmd_callback(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  control_command_received_time_ = this->now();
  control_cmd_ptr_ = msg;
}

void LeoVcuDriver::onEmergencyState(
  autoware_auto_system_msgs::msg::EmergencyState::ConstSharedPtr msg)
{
  is_emergency_ = (msg->state == autoware_auto_system_msgs::msg::EmergencyState::MRM_OPERATING) ||
                  (msg->state == autoware_auto_system_msgs::msg::EmergencyState::MRM_SUCCEEDED) ||
                  (msg->state == autoware_auto_system_msgs::msg::EmergencyState::MRM_FAILED);
  take_over_requested_ =
    msg->state == autoware_auto_system_msgs::msg::EmergencyState::OVERRIDE_REQUESTING;
}

void LeoVcuDriver::emergency_cmd_callback(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  emergency_cmd_ptr = msg;
}

void LeoVcuDriver::gear_cmd_callback(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  gear_cmd_ptr_ = msg;
}

void LeoVcuDriver::turn_indicators_cmd_callback(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_cmd_ptr_ = msg;
}

void LeoVcuDriver::hazard_lights_cmd_callback(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_cmd_ptr_ = msg;
}

void LeoVcuDriver::engage_cmd_callback(
  const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  engage_cmd_ = msg->engage;
}

void LeoVcuDriver::gate_mode_cmd_callback(
  const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  gate_mode_cmd_ptr = msg;  // AUTO = 0, EXTERNAL = 1
}

void LeoVcuDriver::actuator_cmd_callback(
  const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr msg)
{
  actuation_cmd_ptr = msg;
}

void LeoVcuDriver::llc_to_autoware_msg_adapter()
{
  SystemError latest_system_error;

  current_state.twist.longitudinal_velocity =
    static_cast<float>(llc_to_comp_data_.vehicle_dyn_info.linear_vehicle_velocity);
  current_state.steering_wheel_status_msg.data =
    static_cast<float>(llc_to_comp_data_.vehicle_dyn_info.steering_wheel_angle);

  // TODO(ismet): update algorithm for golf vehicle w/ bayram
  current_state.steering_tire_status_msg.steering_tire_angle =
    steering_wheel_to_steering_tire_angle(current_state.steering_wheel_status_msg.data);

  // set hazard lights status
  indicator_adapter_to_autoware(
    static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status.blinker));

  // set gear status
  current_state.gear_report_msg.report = gear_adapter_to_autoware(
    static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status.gear));

  // set control_mode status
  current_state.control_mode_report.mode = control_mode_adapter_to_autoware(
    static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status.mode));

  // set headlight status
  current_state.headlight_msg.report = headlight_adapter_to_autoware(
    static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status.headlight));

  // set handbrake status
  current_state.hand_brake_msg.report =
    static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status.hand_brake);

  // error info msgs
  mechanical_error_check(latest_system_error);
  electrical_error_check(latest_system_error);

  llc_to_state_report_msg_adapter();
  system_error_diagnostics_ = latest_system_error;
}

void LeoVcuDriver::autoware_to_llc_msg_adapter()
{
  if (current_state.gear_report_msg.report != gear_cmd_ptr_->command)
  {
    // velocity is low -> the shift can be changed
    if (std::fabs(current_state.twist.longitudinal_velocity) < gear_shift_velocity_threshold)
    {
      gear_adapter_to_llc(gear_cmd_ptr_->command);
    } else
    {
      RCLCPP_WARN(
        get_logger(), "Gear change is not allowed, current_velocity = %f",
        static_cast<double>(current_state.twist.longitudinal_velocity));
    }
  }
  comp_to_llc_cmd.vehicle_signal_cmd.takeover_request = take_over_requested_ ? 1 : 0;

  // set signal and hazard light status
  indicator_adapter_to_llc();

  // set control mode
  control_mode_adapter_to_llc();

  comp_to_llc_cmd.long_cmd.set_long_accel = control_cmd_ptr_->longitudinal.acceleration;
  comp_to_llc_cmd.long_cmd.set_limit_velocity = control_cmd_ptr_->longitudinal.speed;

  // set headlight and wiper
  comp_to_llc_cmd.vehicle_signal_cmd.headlight = 0;
  comp_to_llc_cmd.vehicle_signal_cmd.wiper = 0;

  comp_to_llc_cmd.vehicle_signal_cmd.long_mode = enable_long_actuation_mode ? 1 : 0;
  if (enable_long_actuation_mode) {
    comp_to_llc_cmd.vehicle_signal_cmd.long_mode = 1;
    comp_to_llc_cmd.long_cmd_actuation.set_gas_pedal_pos = actuation_cmd_ptr->actuation.accel_cmd;
    comp_to_llc_cmd.long_cmd_actuation.set_brake_pedal_pos = actuation_cmd_ptr->actuation.brake_cmd;
  } else {
    comp_to_llc_cmd.vehicle_signal_cmd.long_mode = 0;
    comp_to_llc_cmd.long_cmd_actuation.set_gas_pedal_pos = 0.0;
    comp_to_llc_cmd.long_cmd_actuation.set_brake_pedal_pos = 0.0;
  }

  // TODO(ismet): update transform algorithm (tire -> wheel) for golf
  comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rad =
          steering_tire_to_steering_wheel_angle(control_cmd_ptr_->lateral.steering_tire_angle);
  comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rate =
    steering_tire_to_steering_wheel_angle(control_cmd_ptr_->lateral.steering_tire_angle +
                                          control_cmd_ptr_->lateral.steering_tire_rotation_rate) -
    comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rad;
}

uint8_t LeoVcuDriver::headlight_adapter_to_autoware(uint8_t & input)
{
  if (input == 2) {
    return autoware_auto_vehicle_msgs::msg::HeadlightsReport::ENABLE_LOW;
  } else if (input == 3) {
    return autoware_auto_vehicle_msgs::msg::HeadlightsReport::ENABLE_HIGH;
  } else {
    return autoware_auto_vehicle_msgs::msg::HeadlightsReport::DISABLE;
  }
}

void LeoVcuDriver::control_mode_adapter_to_llc()
{
  /* send mode */
  if (!engage_cmd_) {
    comp_to_llc_cmd.vehicle_signal_cmd.mode = 3;  // DISENGAGED. IT IS PRIOR
  } else if (gate_mode_cmd_ptr->data == tier4_control_msgs::msg::GateMode::AUTO) {
    comp_to_llc_cmd.vehicle_signal_cmd.mode = 1;
  } else if (gate_mode_cmd_ptr->data == tier4_control_msgs::msg::GateMode::EXTERNAL) {
    comp_to_llc_cmd.vehicle_signal_cmd.mode = 2;  // MANUAL
  } else {
    comp_to_llc_cmd.vehicle_signal_cmd.mode = 4;  // NOT READY
  }
}

uint8_t LeoVcuDriver::control_mode_adapter_to_autoware(uint8_t & input)
{
  if (input == 1) {
    return autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  } else if (input == 0 || input == 2) {
    return autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL;
  } else {
    return autoware_auto_vehicle_msgs::msg::ControlModeReport::NO_COMMAND;
  }
}

void LeoVcuDriver::indicator_adapter_to_autoware(uint8_t & input)
{
  if (input == 0) {
    current_state.hazard_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE;
    current_state.turn_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
  } else if (input == 3) {
    current_state.hazard_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE;
    current_state.turn_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
  } else {
    current_state.hazard_msg.report = autoware_auto_vehicle_msgs::msg::HazardLightsReport::DISABLE;
    if (input == 1){
      current_state.turn_msg.report =
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
    } else {
      current_state.turn_msg.report =
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
    }
    }
}

void LeoVcuDriver::indicator_adapter_to_llc()
{
  /* send turn and hazard command */

  if ( hazard_lights_cmd_ptr_->command ==
        autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE)  // It is prior!
  {
    comp_to_llc_cmd.vehicle_signal_cmd.blinker = 3;
  } else if ( turn_indicators_cmd_ptr_->command ==
             autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT)
  {
    comp_to_llc_cmd.vehicle_signal_cmd.blinker = 1;
  } else if ( turn_indicators_cmd_ptr_->command ==
             autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT)
  {
    comp_to_llc_cmd.vehicle_signal_cmd.blinker = 2;
  } else {
    comp_to_llc_cmd.vehicle_signal_cmd.blinker = 0;
  }
}

uint8_t LeoVcuDriver::gear_adapter_to_autoware(
  uint8_t & input)  // TODO(berkay): Check here! Maybe we can make it faster!
{
  switch (input) {
    case 1: // PARK
      return 22;
    case 2: // REVERSE
      return 20;
    case 3: // NEUTRAL
      return 1;
    case 4: // DRIVE
      return 2;
    default: // PARK
      return 22;
  }
}

void LeoVcuDriver::gear_adapter_to_llc(const uint8_t & input)
{
  switch (input) {
    case 1: // NEUTRAL
      comp_to_llc_cmd.vehicle_signal_cmd.gear =  3;
      break;
    case 2: // DRIVE
      comp_to_llc_cmd.vehicle_signal_cmd.gear = 4;
      break;
    case 20: // REVERSE
      if(reverse_gear_enabled_) {
        comp_to_llc_cmd.vehicle_signal_cmd.gear = 2;
      } else {
        comp_to_llc_cmd.vehicle_signal_cmd.gear = 1;
      }
      break;
    case 22: // PARK
      comp_to_llc_cmd.vehicle_signal_cmd.gear = 1;
      break;
    default: // PARK
      comp_to_llc_cmd.vehicle_signal_cmd.gear = 1;
      break;
  }
}

void LeoVcuDriver::llc_to_state_report_msg_adapter()
{
  std_msgs::msg::Header header;
  header.frame_id = this->base_frame_id_;
  header.stamp = get_clock()->now();

  vehicle_state_report_msg_.header = header;

  // Update State Report Msg with Vehicle Status
  vehicle_state_report_msg_.fuel = static_cast<uint8_t>(llc_to_comp_data_.vehicle_sgl_status.fuel);
  vehicle_state_report_msg_.blinker =
    static_cast<uint8_t>(llc_to_comp_data_.vehicle_sgl_status.blinker);
  vehicle_state_report_msg_.headlight =
    static_cast<uint8_t>(llc_to_comp_data_.vehicle_sgl_status.headlight);
  vehicle_state_report_msg_.wiper =
    static_cast<uint8_t>(llc_to_comp_data_.vehicle_sgl_status.wiper);
  vehicle_state_report_msg_.gear = static_cast<uint8_t>(llc_to_comp_data_.vehicle_sgl_status.gear);
  vehicle_state_report_msg_.mode = static_cast<uint8_t>(llc_to_comp_data_.vehicle_sgl_status.mode);
  vehicle_state_report_msg_.hand_brake =
    static_cast<uint8_t>(llc_to_comp_data_.vehicle_sgl_status.hand_brake);
  vehicle_state_report_msg_.horn = static_cast<uint8_t>(llc_to_comp_data_.vehicle_sgl_status.horn);

  // Update State Report Msg with Motion Info
  std::string intervention = std::bitset<8>(llc_to_comp_data_.motion_info.intervention).to_string();
  vehicle_state_report_msg_.steering_intervention = (intervention.at(7) == '1') ? 1 : 0;
  vehicle_state_report_msg_.brake_intervention = (intervention.at(6) == '1') ? 1 : 0;
  vehicle_state_report_msg_.acc_pedal_intervention = (intervention.at(5) == '1') ? 1 : 0;

  vehicle_state_report_msg_.ready = static_cast<uint8_t>(llc_to_comp_data_.motion_info.ready);
  vehicle_state_report_msg_.motion_allow =
    static_cast<uint8_t>(llc_to_comp_data_.motion_info.motion_allow);
  vehicle_state_report_msg_.throttle = static_cast<uint8_t>(llc_to_comp_data_.motion_info.throttle);
  vehicle_state_report_msg_.brake = static_cast<uint8_t>(llc_to_comp_data_.motion_info.brake);
  vehicle_state_report_msg_.front_steer =
    static_cast<uint16_t>(llc_to_comp_data_.motion_info.front_steer);

  // Update State Report Msg with Motor Info
  vehicle_state_report_msg_.motor_temp = static_cast<uint8_t>(llc_to_comp_data_.motor_info.temp);
  vehicle_state_report_msg_.motor_rpm = static_cast<uint16_t>(llc_to_comp_data_.motor_info.rpm);
  vehicle_state_report_msg_.kl75 = static_cast<uint8_t>(llc_to_comp_data_.motor_info.kl75);
}

size_t compare(std::vector<float> & vec, double value)
{
  double dist = std::numeric_limits<double>::max();
  size_t output = 0;
  for (size_t i = 0; i < vec.size(); i++) {
    if (dist > abs(vec.at(i) - value)) {
      dist = abs(vec.at(i) - value);
      output = i;
    }
  }
  return output;
}

float LeoVcuDriver::steering_tire_to_steering_wheel_angle(
  float input)  // rad input degree output, maybe constants needs re-calculation
{               // TODO: If input or output is out of boundary, what we will do?
  input = input - steering_offset;
  float output = 0.0;
  size_t other_idx = 0;
  if (input < steering_angle_.at(0)) {
    input = steering_angle_.at(0);
  }
  if (input > steering_angle_.at(steering_angle_.size() - 1)) {
    input = steering_angle_.at(steering_angle_.size() - 1);
  }

  size_t nearest_idx = compare(steering_angle_, input);

  if (input > steering_angle_.at(nearest_idx)) {
    other_idx = nearest_idx + 1;
  } else if (input < steering_angle_.at(nearest_idx)) {
    other_idx = nearest_idx - 1;
  } else {
    other_idx = nearest_idx;
  }

  if (other_idx == nearest_idx) {
    output = wheel_angle_.at(nearest_idx);
  } else {
    float ratio = (input - steering_angle_.at(nearest_idx)) /
                  (steering_angle_.at(other_idx) - steering_angle_.at(nearest_idx));
    output = wheel_angle_.at(nearest_idx) +
             ratio * (wheel_angle_.at(other_idx) - wheel_angle_.at(nearest_idx));
  }

  return -output;
}

float LeoVcuDriver::steering_wheel_to_steering_tire_angle(
  float input)  // degree input rad output, maybe constants needs re-calculation
{               // TODO: If input or output is out of boundary, what we will do?
  input = -input;
  float output = 0.0;
  size_t other_idx = 0;
  if (input < wheel_angle_.at(0)) {
    input = wheel_angle_.at(0);
  }
  if (input > wheel_angle_.at(wheel_angle_.size() - 1)) {
    input = wheel_angle_.at(wheel_angle_.size() - 1);
  }

  size_t nearest_idx = compare(wheel_angle_, input);

  if (input > wheel_angle_.at(nearest_idx)) {
    other_idx = nearest_idx + 1;
  } else if (input < wheel_angle_.at(nearest_idx)) {
    other_idx = nearest_idx - 1;
  } else {
    other_idx = nearest_idx;
  }

  if (other_idx == nearest_idx) {
    output = steering_angle_.at(nearest_idx);
  } else {
    float ratio = (input - wheel_angle_.at(nearest_idx)) /
                  (wheel_angle_.at(other_idx) - wheel_angle_.at(nearest_idx));
    output = steering_angle_.at(nearest_idx) +
             ratio * (steering_angle_.at(other_idx) - steering_angle_.at(nearest_idx));
  }
  return std::clamp(output + steering_offset,steering_angle_.at(0),steering_angle_.back());
}

void LeoVcuDriver::llc_interface_adapter()
{
  // Update data from LLC
  if (!driver_interface_plugin_->update_received_frame(llc_to_comp_data_)){
    return;
  }
  llc_to_autoware_msg_adapter();

  // To Autoware
  publish_current_vehicle_state();

  bool emergency_send{false};
  bool time_out = false;
  const rclcpp::Time current_time = get_clock()->now();

  if (!autoware_data_ready())
  {
    //TODO(ismet): add what we need to send to CAN when autoware data is not ready
    RCLCPP_WARN_ONCE(get_logger(), "Data from Autoware is not ready!");
    return;
  }

  autoware_to_llc_msg_adapter();

  if (emergency_cmd_ptr->emergency || is_emergency_) {
    if (enable_emergency) {
      emergency_send = true;
    }
  }

  const double control_cmd_delta_time_ms =
    (current_time - control_command_received_time_).seconds() * 1000.0;
  const double t_out = command_timeout_ms_;
  if (t_out >= 0 && control_cmd_delta_time_ms > t_out) {
    time_out = true;
  }

  if (time_out) {
    RCLCPP_ERROR(get_logger(),
      "Emergency Stopping, controller output is timeout = %f ms", control_cmd_delta_time_ms);
    if (enable_cmd_timeout_emergency) {
      emergency_send = true;
    }
  }

  if (emergency_send) {
    comp_to_llc_cmd.vehicle_signal_cmd.takeover_request = 1;
    RCLCPP_ERROR(get_logger(), "~EMERGENCY~\n");
    RCLCPP_ERROR(get_logger(), "Single Point Faults: Emergency hold: %d\n",
                 hazard_status_stamped_->status.emergency_holding);
    for (const auto & diag : hazard_status_stamped_->status.diag_single_point_fault) {
      RCLCPP_ERROR(
        get_logger(),
        "level: %hhu\n"
        "name: %s\n"
        "hardware_id: %s\n"
        "message: %s",
        diag.level, diag.name.c_str(), diag.hardware_id.c_str(), diag.message.c_str());
    }
    RCLCPP_ERROR(get_logger(), "Latent Faults: Emergency hold: %d\n",
                 hazard_status_stamped_->status.emergency_holding);
    for (const auto & diag : hazard_status_stamped_->status.diag_latent_fault) {
      RCLCPP_ERROR(
        get_logger(),
        "level: %hhu\n"
        "name: %s\n"
        "hardware_id: %s\n"
        "message: %s",
        diag.level, diag.name.c_str(), diag.hardware_id.c_str(), diag.message.c_str());
    }
    if (!prev_emergency_) {
      current_emergency_acceleration_ = -std::fabs(soft_stop_acceleration);
      prev_emergency_ = true;
    } else {
      current_emergency_acceleration_ +=
        (1 / data_send_rate_) * (-std::fabs(add_emergency_acceleration_per_second));
    }
    comp_to_llc_cmd.long_cmd.set_long_accel =
      -std::fabs(std::max(-std::fabs(current_emergency_acceleration_),
                          -std::fabs(emergency_stop_acceleration)));
    RCLCPP_ERROR(
      get_logger(),
      "Emergency Stopping, emergency = %d, acceleration = %f, max_acc = %f, soft_acceleration = "
      "%f, acceleration per second = %f\n",
      emergency_cmd_ptr->emergency, comp_to_llc_cmd.long_cmd.set_long_accel,
      emergency_stop_acceleration, soft_stop_acceleration, add_emergency_acceleration_per_second);
  } else {
    prev_emergency_ = false;
    current_emergency_acceleration_ = -std::fabs(soft_stop_acceleration);
    comp_to_llc_cmd.vehicle_signal_cmd.takeover_request = 0;
  }

  // TODO(ismet): update vehicle limits w/ Bayram
  /* check the steering wheel angle and steering wheel angle rate limits */
  if (comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rad < min_steering_wheel_angle ||
      comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rad > max_steering_wheel_angle)
  {
    comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rad = std::min(
      max_steering_wheel_angle, std::max(
                                  comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rad,
                                  min_steering_wheel_angle));
  }

  if ((fabsf(comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rate) >
     max_steering_wheel_angle_rate) && check_steering_angle_rate) {
    comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rate =
      std::min(max_steering_wheel_angle_rate,
               std::max(comp_to_llc_cmd.front_wheel_cmd.set_front_wheel_angle_rate,
                        -max_steering_wheel_angle_rate));
    }

  driver_interface_plugin_->llc_publisher(comp_to_llc_cmd);

  vehicle_state_report_pub_->publish(vehicle_state_report_msg_);

  updater_.force_update();
}

bool LeoVcuDriver::autoware_data_ready()
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  bool output = true;
  if (!control_cmd_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for current_control_cmd ...");
    output = false;
  }
  if (!turn_indicators_cmd_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for turn_indicators_cmd_ ...");
    output = false;
  }
  if (!hazard_lights_cmd_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for hazard_lights_cmd ...");
    output = false;
  }
  if (!gear_cmd_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for gear_cmd ...");
    output = false;
  }
  if (!emergency_cmd_ptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for emergency_cmd ...");
    output = false;
  }
  if (!gate_mode_cmd_ptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for gate_mode_cmd ...");
    output = false;
  }
  if (enable_long_actuation_mode) {
    if (!actuation_cmd_ptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for actuation_mode_cmd ...");
      output = false;
    }
  }

  return output;
}

void LeoVcuDriver::onAutowareState(
  const autoware_auto_system_msgs::msg::AutowareState::SharedPtr message)
{
  using autoware_auto_system_msgs::msg::AutowareState;

  if(message->state == AutowareState::DRIVING){
    comp_to_llc_cmd.vehicle_signal_cmd.hand_brake = 0;
  } else {
    comp_to_llc_cmd.vehicle_signal_cmd.hand_brake = 1;
  }
}

void LeoVcuDriver::publish_current_vehicle_state()
{

  std_msgs::msg::Header header;
  header.frame_id = this->base_frame_id_;
  header.stamp = get_clock()->now();

  /* publish current steering tire status */
  {
    current_state.steering_tire_status_msg.stamp = header.stamp;
    steering_status_pub_->publish(current_state.steering_tire_status_msg);
  }

  /* publish steering wheel status */
  {
    current_state.steering_wheel_status_msg.stamp = header.stamp;
    steering_wheel_status_pub_->publish(current_state.steering_wheel_status_msg);
  }

  /* publish vehicle status control_mode */
  {
    current_state.control_mode_report.stamp = header.stamp;
    control_mode_pub_->publish(current_state.control_mode_report);
  }

  /* publish vehicle status twist */
  {
    current_state.twist.header = header;
    vehicle_twist_pub_->publish(current_state.twist);
  }

  /* publish current shift */
  {
    current_state.gear_report_msg.stamp = header.stamp;
    gear_status_pub_->publish(current_state.gear_report_msg);
  }

  /* publish hazard signal */
  {
    current_state.hazard_msg.stamp = header.stamp;
    hazard_lights_status_pub_->publish(current_state.hazard_msg);
  }

  /* publish turn signal */
  {
    current_state.turn_msg.stamp = header.stamp;
    turn_indicators_status_pub_->publish(current_state.turn_msg);
  }

  /* publish headlight status */
  {
    current_state.hand_brake_msg.stamp = header.stamp;
    hand_brake_pub_->publish(current_state.hand_brake_msg);
  }

  /* publish headlight status */
  {
    current_state.headlight_msg.stamp = header.stamp;
    headlights_pub_->publish(current_state.headlight_msg);
  }
}

void LeoVcuDriver::mechanical_error_check(SystemError & latest_system_error) {
    error_str.data = std::bitset<8>(llc_to_comp_data_.error_info.mechanical_errors).to_string();
    for (size_t i = 0; i < error_str.data.size(); i++) {
        if (error_str.data.at(i) == '0') {
            switch (i) {
                case 0: case 1: case 2: case 3: case 4: case 5: // ignore bits
                    break;
                case 6:
                    RCLCPP_ERROR(this->get_logger(), "Kl75 error.");
                    latest_system_error.kl75_error = true;
                    break;
                case 7:
                    RCLCPP_ERROR(this->get_logger(), "isMotorRunning Error.");
                    latest_system_error.motor_running_error = true;
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid mechanical error message.");
                    break;
            }
        }
    }
    llc_error_pub_->publish(error_str);
}
void LeoVcuDriver::electrical_error_check(SystemError & latest_system_error) {
    error_str.data = std::bitset<16>(llc_to_comp_data_.error_info.electrical_errors).to_string();
    for (size_t i = 0; i < error_str.data.size(); i++) {
        if (error_str.data.at(i) == '1') {
            switch (i) {
                case 4:
                    RCLCPP_ERROR(this->get_logger(), "PC_HeartBeatError");
                    latest_system_error.pc_timeout_error = true;
                    break;
                case 5:
                    RCLCPP_ERROR(this->get_logger(), "Brake_HeartBeatError");
                    latest_system_error.brake_timeout_error = true;
                    break;
                case 6:
                    RCLCPP_ERROR(this->get_logger(), "Brake_SystemError");
                    latest_system_error.brake_system_error = true;
                    break;
                case 7:
                    RCLCPP_ERROR(this->get_logger(), "EPAS_HeartBeatError");
                    latest_system_error.epas_timeout_error = true;
                    break;
                case 8:
                    RCLCPP_ERROR(this->get_logger(), "EPAS_SystemError");
                    latest_system_error.epas_system_error = true;
                    break;
                case 9:
                    // RCLCPP_ERROR(this->get_logger(), "G29_HeartBeatError");
                    latest_system_error.g29_timeout_error = true;
                    break;
                case 10:
                    RCLCPP_ERROR(this->get_logger(), "Throttle_ECU_HeartBeatError");
                    latest_system_error.throttle_ecu_timeout_error = true;
                    break;
                case 11:
                    RCLCPP_ERROR(this->get_logger(), "BrakePowerError");
                    latest_system_error.brake_power_error = true;
                    break;
                case 12:
                    RCLCPP_ERROR(this->get_logger(), "EPASPowerError");
                    latest_system_error.epas_power_error = true;
                    break;
                case 13:
                    RCLCPP_ERROR(this->get_logger(), "By_WirePowerError");
                    latest_system_error.by_wire_power_error = true;
                    break;
                case 14:
                    RCLCPP_ERROR(this->get_logger(), "PDS_BusError");
                    latest_system_error.pds_bus_error = true;
                    break;
                case 15:
                    //RCLCPP_ERROR(this->get_logger(), "PDS_HearBeatError");
                    latest_system_error.pds_timeout_error = true;
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid error message.");
                    break;
            }
        }
    }
    llc_error_pub_->publish(error_str);
}

void LeoVcuDriver::checkMotorRunningError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("motor_running_error", system_error_diagnostics_.motor_running_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "Motor Running system works as expected";
    if (system_error_diagnostics_.motor_running_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "Motor running error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkKl75Error(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("kl75_error", system_error_diagnostics_.kl75_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "Kl75 works as expected";
    if (system_error_diagnostics_.kl75_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "Kl75 error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkPDSTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("pds_timeout_error", system_error_diagnostics_.pds_timeout_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "PDS communication works as expected";
    if (system_error_diagnostics_.pds_timeout_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "PDS timeout error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkPDSBusError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("pds_bus_error", system_error_diagnostics_.pds_bus_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "PDS bus works as expected";
    if (system_error_diagnostics_.pds_bus_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "PDS bus error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkByWireError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("by_wire_power_error", system_error_diagnostics_.by_wire_power_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "By wire system powered up";
    if (system_error_diagnostics_.by_wire_power_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "By wire system power error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkEPASPowerError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("epas_power_error", system_error_diagnostics_.epas_power_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "EPAS system powered up";
    if (system_error_diagnostics_.epas_power_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "EPAS power error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkBrakePowerError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("brake_power_error", system_error_diagnostics_.brake_power_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "Brake system powered up";
    if (system_error_diagnostics_.brake_power_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "Brake system power error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkThrottleTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("throttle_ecu_timeout_error", system_error_diagnostics_.throttle_ecu_timeout_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "Throttle-ECU system communication works as expected";
    if (system_error_diagnostics_.throttle_ecu_timeout_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "Throttle-ECU system timeout error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkG29TimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("g29_timeout_error", system_error_diagnostics_.g29_timeout_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "G29 communication works as expected";
    if (system_error_diagnostics_.g29_timeout_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "G29 timeout error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkEPASSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("epas_system_error", system_error_diagnostics_.epas_system_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "EPAS system works as expected";
    if (system_error_diagnostics_.epas_system_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "EPAS system error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkEPASTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("epas_timeout_error", system_error_diagnostics_.epas_timeout_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "EPAS communication works as expected";
    if (system_error_diagnostics_.epas_timeout_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "EPAS timeout error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkBrakeSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("brake_system_error", system_error_diagnostics_.brake_system_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "Brake system works as expected";
    if (system_error_diagnostics_.brake_system_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "Brake system error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkBrakeTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("brake_timeout_error", system_error_diagnostics_.brake_timeout_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "Brake communication system works as expected";
    if (system_error_diagnostics_.brake_timeout_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "Brake system timeout error detected";
    }
    stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkPCTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    stat.add("pc_timeout_error", system_error_diagnostics_.pc_timeout_error);
    int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "PC communication system works as expected";
    if (system_error_diagnostics_.pc_timeout_error) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = "PC system timeout error detected";
    }
    stat.summary(diag_level, diag_message);
}
