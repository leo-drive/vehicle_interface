// Copyright 2022 Leo Drive Teknoloji A.Ş.
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

/// \copyright Copyright 2022 Leo Drive Teknoloji A.Ş.
/// \file
/// \brief This file defines the vehicle_interface class.
#ifndef BUILD_VEHICLE_INTERFACE_H
#define BUILD_VEHICLE_INTERFACE_H

#include <cstdint>
#include "checksum.h"


using namespace std;

const uint8_t comp_to_llc_msg_frame_id {16U};
const uint8_t comp_to_llc_msg_eof_id {17U};

const uint8_t llc_to_comp_msg_frame_id {99U};
const uint8_t llc_to_comp_msg_eof_id {100U};

// added for universe
struct vehicle_current_state_
{
  autoware_auto_vehicle_msgs::msg::VelocityReport twist;
  tier4_vehicle_msgs::msg::SteeringWheelStatusStamped steering_wheel_status_msg;
  autoware_auto_vehicle_msgs::msg::SteeringReport steering_tire_status_msg;
  autoware_auto_vehicle_msgs::msg::ControlModeReport control_mode_report;
  autoware_auto_vehicle_msgs::msg::GearReport gear_report_msg;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
  autoware_auto_vehicle_msgs::msg::HazardLightsReport hazard_msg;
  autoware_auto_vehicle_msgs::msg::HandBrakeReport hand_brake_msg;
  autoware_auto_vehicle_msgs::msg::HeadlightsReport headlight_msg;
  char * debug_str_last {};
};

struct StateReport_
{
  uint8_t fuel;
  uint8_t blinker;
  uint8_t headlight;
  uint8_t wiper;
  uint8_t gear;
  uint8_t mode;
  uint8_t hand_brake;
  uint8_t takeover_request;
  uint8_t intervention;
  uint8_t ready;
  uint8_t motion_allow;
  uint8_t throttle;          // %
  uint8_t brake;           // %
  int16_t front_steer;           //degree
  char debugstr[24];
};

//can msgs
struct LlcCanMsg
{
    can_msgs::msg::Frame msg_long_cmd_frame_1;
    can_msgs::msg::Frame msg_long_cmd_frame_2;
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
  uint8_t k157;
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
  float linear_veh_velocity;
  float steering_wheel_angle;
};

//can commands

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

struct __attribute__((packed)) LongitudinalCommandDataV2{
  float set_gas_pedal_pos;
  float set_brake_pedal_pos;
};

struct __attribute__((packed)) LongitudinalCommandDataV1{
  float set_long_accel;
  float set_limit_velocity;
};

struct __attribute__((packed)) LlcToCompData {
    VehicleErrorsData err_msg;
    MotorInfoData motor_info_msg;
    MotionInfoData motion_info_;
    VehicleSignalStatusData vehicle_sgl_status_;
    VehicleDynamicsInfoData vehicle_dyn_info_;
    StateReport_ state_report_;
};

struct __attribute__((packed)) CompToLlcCmd {
    FrontWheelCommandData front_wheel_cmd_msg;
    VehicleSignalCommandData vehicle_signal_cmd;
    LongitudinalCommandDataV1 long_msg_v1;
    LongitudinalCommandDataV2 long_msg_v2;
};



#endif //BUILD_VEHICLE_INTERFACE_H
