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
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo()),
  updater_(this)
{
  /// Get Params Here!
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000.0);
  loop_rate_ = declare_parameter("loop_rate", 100.0);
  /* parameters for vehicle specifications */
  wheel_base_ = static_cast<float>(vehicle_info_.wheel_base_m);
  reverse_gear_enabled_ = declare_parameter("reverse_gear_enabled", false);
  emergency_stop_acceleration =
    static_cast<float>(declare_parameter("emergency_stop_acceleration", -5.0));
  gear_shift_velocity_threshold =
    static_cast<float>(declare_parameter("gear_shift_velocity_threshold", 0.1));
  max_steering_wheel_angle =
    static_cast<float>(declare_parameter("max_steering_wheel_angle", 750.0));
  min_steering_wheel_angle =
    static_cast<float>(declare_parameter("min_steering_wheel_angle", -750.0));
  max_steering_wheel_angle_rate =
    static_cast<float>(declare_parameter("max_steering_wheel_angle_rate", 300.0));
  check_steering_angle_rate = declare_parameter("check_steering_angle_rate", true);
  soft_stop_acceleration = static_cast<float>(declare_parameter("soft_stop_acceleration", -1.5));
  add_emergency_acceleration_per_second =
    static_cast<float>(declare_parameter("add_emergency_acceleration_per_second", -0.5));
  enable_emergency = declare_parameter("enable_emergency", true);
  enable_cmd_timeout_emergency = declare_parameter("enable_cmd_timeout_emergency", true);
  enable_debugger = declare_parameter("enable_debugger", true);
  steering_offset = static_cast<float>(declare_parameter("steering_offset", 0.0));

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
      "/system/emergency/hazard_lights_cmd", rclcpp::QoS{1},
      std::bind(&LeoVcuDriver::hazard_lights_cmd_callback, this, _1));
  engage_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    "/vehicle/engage", rclcpp::QoS{1}, std::bind(&LeoVcuDriver::engage_cmd_callback, this, _1));
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

  /* hand_brake_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>(
          "/yatopicyok", rclcpp::QoS{1},
          std::bind(&LeoVcuDriver::hand_brake_cmd_callback, this, _1));
  headlights_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::HeadlightsCommand>(
          "/neolacakhalimiz", rclcpp::QoS{1},
          std::bind(&LeoVcuDriver::headlights_cmd_callback, this, _1));
  raw_control_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::RawControlCommand>(
          "/kirlisepetimnoktacom", rclcpp::QoS{1},
          std::bind(&LeoVcuDriver::raw_control_cmd_callback, this, _1)); */
  /* publisher */

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
  // System error diagnostic
  updater_.setHardwareID("vehicle_error_monitor");
  updater_.add("pds_system_error", this, &LeoVcuDriver::checkPDSSystemError);
  updater_.add("bbw_system_error", this, &LeoVcuDriver::checkBBWSystemError);
  updater_.add("epas_system_error", this, &LeoVcuDriver::checkEPASSystemError);
  updater_.add("pc_ignition_error", this, &LeoVcuDriver::checkPCIgnitionError);
  updater_.add("epas_pwr_error", this, &LeoVcuDriver::checkEPASPowerError);
  updater_.add("sbw_pwr_error", this, &LeoVcuDriver::checkSBWPowerError);
  updater_.add("bbw_pwr_error", this, &LeoVcuDriver::checkBBWPowerError);
  updater_.add("pc_timeout", this, &LeoVcuDriver::checkPCTimeoutError);
  updater_.add("bcu_timeout", this, &LeoVcuDriver::checkBCUTimeoutError);
  updater_.add("pds_timeout", this, &LeoVcuDriver::checkPDSTimeoutError);
  updater_.add("epas_timeout", this, &LeoVcuDriver::checkEPASTimeoutError);
  updater_.add("bbw_timeout", this, &LeoVcuDriver::checkBBWTimeoutError);

  // Timer
  loop_rate_ = 25;
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  tim_data_sender_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&LeoVcuDriver::llc_publisher, this));
  current_emergency_acceleration = -std::fabs(soft_stop_acceleration);

  sub_recv_frame_ = this->create_subscription<can_msgs::msg::Frame>(
          "/from_can_bus", rclcpp::QoS{1}, std::bind(&LeoVcuDriver::receivedFrameCallback, this, _1));

  pub_send_frame_ = this->create_publisher<can_msgs::msg::Frame>(
          "/to_can_bus", rclcpp::QoS{10});

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
  if(enable_debugger){
    RCLCPP_INFO(
      get_logger(), "target steering degree: %f",
      control_cmd_ptr_->lateral.steering_tire_angle * 180.0 / M_PI);
  }
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

  /* void LeoVcuDriver::hand_brake_cmd_callback(
          const autoware_auto_vehicle_msgs::msg::HandBrakeCommand::ConstSharedPtr msg)
  {
      hand_brake_cmd_ptr = msg;
  }

  void LeoVcuDriver::headlights_cmd_callback(
          const autoware_auto_vehicle_msgs::msg::HeadlightsCommand::ConstSharedPtr  msg)
  {
      head_lights_cmd_ptr = msg;
  }

  void LeoVcuDriver::raw_control_cmd_callback(
          const autoware_auto_vehicle_msgs::msg::RawControlCommand::ConstSharedPtr msg)
  {
      raw_control_cmd_ptr = msg;
  } */

void LeoVcuDriver::llc_to_autoware_msg_adapter()
{
  switch (msg_recv_can_frame_->id) {
    case 1041: // get linear vehicle velocity and front wheel angle
      {
        std::memcpy(&llc_to_comp_data_.vehicle_dyn_info_, &msg_recv_can_frame_->data, 8);
        current_state.twist.longitudinal_velocity =
          static_cast<float>(llc_to_comp_data_.vehicle_dyn_info_.linear_veh_velocity);
        current_state.steering_wheel_status_msg.data =
          static_cast<float>(llc_to_comp_data_.vehicle_dyn_info_.steering_wheel_angle);
        // TODO(ismet): update algorithm for golf vehicle
        current_state.steering_tire_status_msg.steering_tire_angle =
          steering_wheel_to_steering_tire_angle(current_state.steering_wheel_status_msg.data);
      }
      break;

    case 1042: // fuel, blinker, headlight, wiper, gear, mode, hand_brake and horn publisher
      {
        std::memcpy(&llc_to_comp_data_.vehicle_sgl_status_, &msg_recv_can_frame_->data, 8);

        // set hazard lights status
        indicator_adapter_to_autoware(
          static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status_.blinker));

        // set gear status
        current_state.gear_report_msg.report = gear_adapter_to_autoware(
          static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status_.gear));

        // set control_mode status
        current_state.control_mode_report.mode = control_mode_adapter_to_autoware(
          static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status_.mode));

        // set headlight status
        current_state.headlight_msg.report = headlight_adapter_to_autoware(
          static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status_.headlight));

        // set handbrake status
        current_state.hand_brake_msg.report =
          static_cast<uint8_t&>(llc_to_comp_data_.vehicle_sgl_status_.hand_brake);
      }
      break;
    case 1043: // intervention, ready, motion_allow, throttle, brake, front_steer
      {
        std::memcpy(&llc_to_comp_data_.motion_info_, &msg_recv_can_frame_->data, 8);
      }
      break;
    case 1044: // temperature and rpm
      {
      std::memcpy(&llc_to_comp_data_.motor_info_msg, &msg_recv_can_frame_->data, 8);
      }
      break;
    case 1045:
      {
        std::memcpy(&llc_to_comp_data_.err_msg, &msg_recv_can_frame_->data, 8);
        // error info msgs
        mechanical_error_check();
        electrical_error_check();
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid CanId\n");
      break;
  }
}

void LeoVcuDriver::autoware_to_llc_msg_adapter()
{
  if (current_state.gear_report_msg.report != gear_cmd_ptr_->command)
  {
    // velocity is low -> the shift can be changed
    if (std::fabs(current_state.twist.longitudinal_velocity) < gear_shift_velocity_threshold)
    {
      // TODO(berkay): check here again!
      gear_adapter_to_llc(gear_cmd_ptr_->command);
    } else
    {
      RCLCPP_WARN(
        get_logger(), "Gear change is not allowed, current_velocity = %f",
        static_cast<double>(current_state.twist.longitudinal_velocity));
    }
  }
  comp_to_llc_cmd.vehicle_signal_cmd.takeover_request = take_over_requested_ ? 1 : 0;

  indicator_adapter_to_llc();

  control_mode_adapter_to_llc();

  comp_to_llc_cmd.long_msg_v1.set_long_accel = control_cmd_ptr_->longitudinal.acceleration;
  comp_to_llc_cmd.long_msg_v1.set_limit_velocity = control_cmd_ptr_->longitudinal.speed;

  // TODO(ismet): update algorithm for following commands
  comp_to_llc_cmd.vehicle_signal_cmd.headlight = 0;
  comp_to_llc_cmd.vehicle_signal_cmd.wiper = 0;
  comp_to_llc_cmd.vehicle_signal_cmd.long_mode = 0;

  comp_to_llc_cmd.long_msg_v2.set_gas_pedal_pos = 0.0;
  // static_cast<float>(raw_control_cmd_ptr->throttle) / 100;
  comp_to_llc_cmd.long_msg_v2.set_brake_pedal_pos = 0.0;
  // static_cast<float>(raw_control_cmd_ptr->brake) / 100;


  // TODO(ismet): update transform algorithm (tire -> wheel) for golf
  comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rad =
          steering_tire_to_steering_wheel_angle(control_cmd_ptr_->lateral.steering_tire_angle);
  comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rate =
          steering_tire_to_steering_wheel_angle(control_cmd_ptr_->lateral.steering_tire_angle +
                                            control_cmd_ptr_->lateral.steering_tire_rotation_rate) -
                                            comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rad;
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
      current_state.turn_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
    } else {
      current_state.turn_msg.report = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
    }
    }
}

void LeoVcuDriver::indicator_adapter_to_llc()
{
  /* send turn and hazard commad */

  if ( hazard_lights_cmd_ptr_->command == autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE)  // It is prior!
  {
    comp_to_llc_cmd.vehicle_signal_cmd.blinker = 3;
  } else if ( turn_indicators_cmd_ptr_->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT)
  {
    comp_to_llc_cmd.vehicle_signal_cmd.blinker = 1;
  } else if ( turn_indicators_cmd_ptr_->command == autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT)
  {
    comp_to_llc_cmd.vehicle_signal_cmd.blinker = 2;
  } else {
    comp_to_llc_cmd.vehicle_signal_cmd.blinker = 0;
  }
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
{               // TODO: If input or output is out of boundry, what we will do?
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
{               // TODO: If input or output is out of boundry, what we will do?
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

void LeoVcuDriver::llc_publisher()
{
  std_msgs::msg::Header header;
  header.frame_id = this->base_frame_id_;
  header.stamp = get_clock()->now();
  bool emergency_send{false};

  if(pub_send_frame_->get_subscription_count() < 1)
  {
    RCLCPP_WARN(
      get_logger(), "CAN RECEIVER IS NOT READY!");
    return;
  }
  if (!autoware_data_ready())
  {
    //TODO(ismet): add what we need to send to CAN when autoware data is not ready
    RCLCPP_WARN_ONCE(get_logger(), "Data from Autoware is not ready!");
    return;
  }

  autoware_to_llc_msg_adapter();
  //TODO(ismet): add emergency handling

  /* check the steering wheel angle and steering wheel angle rate limits */
  if (comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rad < min_steering_wheel_angle ||
      comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rad > max_steering_wheel_angle)
  {
    comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rad = std::min( max_steering_wheel_angle,
               std::max(comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rad, min_steering_wheel_angle));
  }

  if ((fabsf(comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rate) > max_steering_wheel_angle_rate) && check_steering_angle_rate) {
    comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rate = std::min(max_steering_wheel_angle_rate,
            std::max(comp_to_llc_cmd.front_wheel_cmd_msg.set_front_wheel_angle_rate, -max_steering_wheel_angle_rate));
    }

  std::memcpy(&llc_can_msgs.msg_long_cmd_frame_1.data, &comp_to_llc_cmd.long_msg_v1, 8);
  std::memcpy(&llc_can_msgs.msg_long_cmd_frame_2.data, &comp_to_llc_cmd.long_msg_v2, 8);
  std::memcpy(&llc_can_msgs.msg_veh_signal_cmd_frame.data, &comp_to_llc_cmd.vehicle_signal_cmd, 8);
  std::memcpy(&llc_can_msgs.msg_front_wheel_cmd_frame.data, &comp_to_llc_cmd.front_wheel_cmd_msg, 8);

  //sendCanFrame();

  llc_can_msgs.msg_long_cmd_frame_1.set__header(header);
  llc_can_msgs.msg_long_cmd_frame_1.dlc = static_cast<uint8_t>(8);
  llc_can_msgs.msg_long_cmd_frame_1.id = static_cast<uint32_t>(1024);

  llc_can_msgs.msg_long_cmd_frame_2.set__header(header);
  llc_can_msgs.msg_long_cmd_frame_2.set__dlc(static_cast<uint8_t>(8));
  llc_can_msgs.msg_long_cmd_frame_2.set__id(static_cast<uint32_t>(1025));

  llc_can_msgs.msg_veh_signal_cmd_frame.set__header(header);
  llc_can_msgs.msg_veh_signal_cmd_frame.set__dlc(static_cast<uint8_t>(8));
  llc_can_msgs.msg_veh_signal_cmd_frame.set__id(static_cast<uint32_t>(1026));

  llc_can_msgs.msg_front_wheel_cmd_frame.set__header(header);
  llc_can_msgs.msg_front_wheel_cmd_frame.set__dlc(static_cast<uint8_t>(8));
  llc_can_msgs.msg_front_wheel_cmd_frame.set__id(static_cast<uint32_t>(1027));

  pub_send_frame_->publish(llc_can_msgs.msg_long_cmd_frame_1);
  pub_send_frame_->publish(llc_can_msgs.msg_long_cmd_frame_2);
  pub_send_frame_->publish(llc_can_msgs.msg_veh_signal_cmd_frame);
  pub_send_frame_->publish(llc_can_msgs.msg_front_wheel_cmd_frame);
}

uint8_t LeoVcuDriver::gear_adapter_to_autoware(
  uint8_t & input)  // TODO(berkay): Check here! Maybe we can make it faster!
{
  switch (input) {
    case 1:
      return 2;
    case 2:
      return 20;
    case 3:
      return 22;
    case 4:
      return 23;
    case 5:
      return 1;
    default:
      return 0;
  }
}

void LeoVcuDriver::gear_adapter_to_llc(const uint8_t & input)
{
  if (input <= 19 && input >= 2) {
      comp_to_llc_cmd.vehicle_signal_cmd.gear = 1;
  } else if (input == 20) {
    if (reverse_gear_enabled_) {
      comp_to_llc_cmd.vehicle_signal_cmd.gear = 2;
    } else {
      comp_to_llc_cmd.vehicle_signal_cmd.gear = 0;
    }
  } else if (input == 22) {
    comp_to_llc_cmd.vehicle_signal_cmd.gear = 3;
  } else if (input == 23) {
    comp_to_llc_cmd.vehicle_signal_cmd.gear = 4;
  } else if (input == 1) {
    comp_to_llc_cmd.vehicle_signal_cmd.gear = 5;
  } else {
    comp_to_llc_cmd.vehicle_signal_cmd.gear = 0;
  }
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

  return output;
}
void LeoVcuDriver::checkPDSSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("pds_system_error", system_error_diagnostics_.pds_system_error);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "PDS system works as expected";
  if (system_error_diagnostics_.pds_system_error) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "PDS system error detected";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkBBWSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("bbw_system_error", system_error_diagnostics_.bbw_system_error);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "BBW system works as expected";
  if (system_error_diagnostics_.bbw_system_error) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "BBW system error detected";
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
void LeoVcuDriver::checkPCIgnitionError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("pc_ignition_error", system_error_diagnostics_.pc_ignition_error);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "PC ignitions received";
  if (system_error_diagnostics_.pc_ignition_error) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "PC ignition has not been received";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkEPASPowerError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("epas_pwr_error", system_error_diagnostics_.epas_pwr_error);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "EPAS system powered up";
  if (system_error_diagnostics_.epas_pwr_error) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "EPAS power error detected";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkSBWPowerError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("sbw_pwr_error", system_error_diagnostics_.sbw_pwr_error);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "SBW system powered up";
  if (system_error_diagnostics_.sbw_pwr_error) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "SBW power error detected";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkBBWPowerError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("bbw_pwr_error", system_error_diagnostics_.bbw_pwr_error);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "BBW system powered up";
  if (system_error_diagnostics_.bbw_pwr_error) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "BBW power error deteced";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkPCTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("pc_timeout", system_error_diagnostics_.pc_timeout);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "PC communication works as expected";
  if (system_error_diagnostics_.pc_timeout) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "PC timeout";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkBCUTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("bcu_timeout", system_error_diagnostics_.bcu_timeout);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "Volt BCU communication works as expected";
  if (system_error_diagnostics_.bcu_timeout) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "Volt BCU timeout";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkPDSTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("pds_timeout", system_error_diagnostics_.pds_timeout);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "LEO PDS communication works as expected";
  if (system_error_diagnostics_.pds_timeout) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "LEO PDS timeout";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkEPASTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("epas_timeout", system_error_diagnostics_.epas_timeout);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "EPAS communication works as expected";
  if (system_error_diagnostics_.epas_timeout) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "EPAS timeout";
  }
  stat.summary(diag_level, diag_message);
}
void LeoVcuDriver::checkBBWTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("bbw_timeout", system_error_diagnostics_.bbw_timeout);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "BBW communication works as expected";
  if (system_error_diagnostics_.bbw_timeout) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "BBW timeout";
  }
  stat.summary(diag_level, diag_message);
}

void LeoVcuDriver::onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::SharedPtr message)
{
  using autoware_auto_system_msgs::msg::AutowareState;

  if(message->state == AutowareState::DRIVING){
    comp_to_llc_cmd.vehicle_signal_cmd.hand_brake = 0;
  } else {
    comp_to_llc_cmd.vehicle_signal_cmd.hand_brake = 1;
  }
}

void LeoVcuDriver::receivedFrameCallback(can_msgs::msg::Frame::SharedPtr msg) {
  msg_recv_can_frame_ = msg;

  std_msgs::msg::Header header;
  header.frame_id = this->base_frame_id_;
  header.stamp = get_clock()->now();

  /* convert received data to autoware msgs */
  llc_to_autoware_msg_adapter();

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

void LeoVcuDriver::mechanical_error_check() {
    error_str.data = std::bitset<8>(llc_to_comp_data_.err_msg.mechanical_errors).to_string();
    for (size_t i = 0; i < error_str.data.size(); i++) {
        if (error_str.data.at(i) == '1') {
            switch (i) {
                case 6:
                    RCLCPP_ERROR(this->get_logger(), "isMotorRunning Error.");
                    break;
                case 7:
                    RCLCPP_ERROR(this->get_logger(), "K175 error.");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid mehcanical error message.");
                    break;
            }
        }
    }
    llc_error_pub_->publish(error_str);
}

void LeoVcuDriver::electrical_error_check() {
    error_str.data = std::bitset<16>(llc_to_comp_data_.err_msg.electrical_errors).to_string();
    for (size_t i = 0; i < error_str.data.size(); i++) {
        if (error_str.data.at(i) == '1') {
            switch (i) {
                case 5:
                    RCLCPP_ERROR(this->get_logger(), "PDS_HearBeatError");
                    break;
                case 6:
                    RCLCPP_ERROR(this->get_logger(), "PDS_BusError");
                    break;
                case 7:
                    RCLCPP_ERROR(this->get_logger(), "By_WirePowerError");
                    break;
                case 8:
                    RCLCPP_ERROR(this->get_logger(), "EPASPowerError");
                    break;
                case 9:
                    RCLCPP_ERROR(this->get_logger(), "BrakePowerError");
                    break;
                case 10:
                    RCLCPP_ERROR(this->get_logger(), "Throttle_ECU_HeartBeatError");
                    break;
                case 11:
                    RCLCPP_ERROR(this->get_logger(), "G29_HeartBeatEror");
                    break;
                case 12:
                    RCLCPP_ERROR(this->get_logger(), "EPAS_SystemError");
                    break;
                case 13:
                    RCLCPP_ERROR(this->get_logger(), "Brake_SystemError");
                    break;
                case 14:
                    RCLCPP_ERROR(this->get_logger(), "Brake_HeartBeatError");
                    break;
                case 15:
                    RCLCPP_ERROR(this->get_logger(), "PC_HeartBeatError");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Invalid error message.");
                    break;
            }
        }
    }
    llc_error_pub_->publish(error_str);
}
