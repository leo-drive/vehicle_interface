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

/// \copyright Copyright 2023 Leo Drive Teknoloji A.Ş.
/// \file
/// \brief This file defines the vehicle_interface class.
#ifndef BUILD_VEHICLE_INTERFACE_H
#define BUILD_VEHICLE_INTERFACE_H


#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_system_msgs/msg/hazard_status_stamped.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>

#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/hand_brake_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/headlights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/headlights_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/raw_control_command.hpp>

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>


#include <std_msgs/msg/string.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>

#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>

#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <leo_vcu_msgs/msg/state_report.hpp>

#include <can_msgs/msg/frame.hpp>

#include <cstdint>

namespace leo_vcu_driver::vehicle_interface
{

using namespace std;

// Added for universe
struct vehicle_current_state_
{
  autoware_auto_vehicle_msgs::msg::VelocityReport twist;
  tier4_vehicle_msgs::msg::SteeringWheelStatusStamped steering_wheel_status_msg;
  tier4_vehicle_msgs::msg::ActuationStatusStamped actuation_status_msg;
  autoware_auto_vehicle_msgs::msg::SteeringReport steering_tire_status_msg;
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_report;
  autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
  autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;
  autoware_auto_vehicle_msgs::msg::HandBrakeReport hand_brake_msg;
  autoware_auto_vehicle_msgs::msg::HeadlightsReport headlight_msg;
};

// Added for to store autonomous system faults
struct SystemError
{
  bool motor_running_error = false;
  bool kl75_error = false;
  bool pds_timeout_error = false;
  bool pds_bus_error = false;
  bool by_wire_power_error = false;
  bool epas_power_error = false;
  bool brake_power_error = false;
  bool throttle_ecu_timeout_error = false;
  bool g29_timeout_error = false;
  bool epas_system_error = false;
  bool epas_timeout_error = false;
  bool brake_system_error = false;
  bool brake_timeout_error = false;
  bool pc_timeout_error = false;
};

// Can msgs
struct LlcCanMsg
{
  can_msgs::msg::Frame long_cmd_msg;
  can_msgs::msg::Frame long_actuation_cmd_msg;
  can_msgs::msg::Frame msg_veh_signal_cmd_frame;
  can_msgs::msg::Frame msg_front_wheel_cmd_frame;
};

struct __attribute__((packed)) VehicleErrorsData {
  uint8_t mechanical_errors;
  uint16_t electrical_errors;
};

struct __attribute__((packed)) MotorInfoData{
  uint8_t temp;
  uint16_t rpm;
  uint8_t kl75;
};

struct __attribute__((packed)) MotionInfoData{
  uint8_t intervention;
  uint8_t ready;
  uint8_t motion_allow;
  uint8_t throttle;
  uint8_t brake;
  uint16_t front_steer;
};

struct __attribute__((packed)) VehicleSignalStatusData{
  uint8_t fuel;
  uint8_t blinker;
  uint8_t headlight;
  uint8_t wiper;
  uint8_t gear;
  uint8_t mode;
  uint8_t hand_brake;
  uint8_t horn;
};

struct __attribute__((packed)) VehicleDynamicsInfoData{
  float linear_vehicle_velocity;
  float steering_wheel_angle;
};

// Can commands
struct __attribute__((packed)) FrontWheelCommandData{
  float set_front_wheel_angle_rad;
  float set_front_wheel_angle_rate;
};

struct __attribute__((packed)) VehicleSignalCommandData{
  uint8_t blinker;
  uint8_t headlight;
  uint8_t wiper;
  uint8_t gear;
  uint8_t mode;
  uint8_t hand_brake;
  uint8_t takeover_request;
  uint8_t long_mode;
};

struct __attribute__((packed)) LongitudinalCommandDataActuation{
  float set_gas_pedal_pos;
  float set_brake_pedal_pos;
};

struct __attribute__((packed)) LongitudinalCommandData{
  float set_long_accel;
  float set_limit_velocity;
};

// From LLC
struct __attribute__((packed)) LlcToCompData {
  /* Serial Frame and Counter Info */
  uint8_t frame_id1;
  uint8_t frame_id2;
  uint32_t counter;

  /* Serial and CAN Common Data */
  VehicleErrorsData error_info;
  MotorInfoData motor_info;
  MotionInfoData motion_info;
  VehicleSignalStatusData vehicle_sgl_status;
  VehicleDynamicsInfoData vehicle_dyn_info;

  /* Serial CRC Check and EOF ID Info */
  uint16_t crc;
  uint8_t eof_id1;
  uint8_t eof_id2;
};

// To LLC
struct __attribute__((packed)) CompToLlcCmd {
  /* Serial Frame and Counter Cmd */
  uint8_t frame_id1;
  uint8_t frame_id2;
  uint32_t counter{0};

  /* Serial and CAN Common Data Cmd*/
  FrontWheelCommandData front_wheel_cmd;
  VehicleSignalCommandData vehicle_signal_cmd;
  LongitudinalCommandData long_cmd;
  LongitudinalCommandDataActuation long_cmd_actuation;

  /* Serial CRC and EOF ID Cmd */
  uint16_t crc{0};
  uint8_t eof_id1;
  uint8_t eof_id2;
};
}

#endif //BUILD_VEHICLE_INTERFACE_H
