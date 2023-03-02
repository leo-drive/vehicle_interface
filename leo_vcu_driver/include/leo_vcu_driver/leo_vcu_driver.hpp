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
//
// Co-developed by Tier IV, Inc. Apex.AI, Inc. and Leo Drive Teknoloji A.Ş.

/// \copyright Copyright 2022 Leo Drive Teknoloji A.Ş.
/// \file
/// \brief This file defines the leo_vcu_driver class.

#ifndef LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_
#define LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <experimental/optional>
#include <leo_vcu_driver/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

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
//mehce added starts
#include <autoware_auto_vehicle_msgs/msg/hand_brake_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hand_brake_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/headlights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/headlights_report.hpp>

#include <autoware_auto_vehicle_msgs/msg/raw_control_command.hpp>

//#include <pacmod3_msgs/pacmod3_msgs/msg/accel_aux_rpt.hpp>
//mehce aded ends
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>

#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>

#include <tier4_vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <can_msgs/msg/frame.hpp>
//#include <leo_vcu_driver/AsyncSerial.h>
#include <leo_vcu_driver/checksum.h>
#include <leo_vcu_driver/vehicle_interface.h>
#include <linux/can.h>
#include <bitset>
#include <string>
#include <vector>

class LeoVcuDriver : public rclcpp::Node
{
public:
  LeoVcuDriver();
  ~LeoVcuDriver() override { /*;serial->close();*/ }

  /**
   * @brief It checks the autoware data is ready or not.
   */
  bool autoware_data_ready();
  /**
   * @brief It is callback function which takes data from "/control/command/control_cmd" topic in
   * Autoware Universe.
   */
  void ctrl_cmd_callback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/control/command/turn_indicators_cmd"
   * topic in Autoware Universe.
   */
  void turn_indicators_cmd_callback(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/control/command/hazard_lights_cmd" topic
   * from Autoware Universe.
   */
  void hazard_lights_cmd_callback(
    const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/control/command/gear_cmd" topic from
   * Autoware Universe.
   */
  void gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/vehicle/engage" topic from Autoware
   * Universe.
   */
  void engage_cmd_callback(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/control/command/emergency_cmd" topic
   * from Autoware Universe.
   */
  void emergency_cmd_callback(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/control/current_gate_mode" topic from
   * Autoware Universe.
   */
  void gate_mode_cmd_callback(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg);
   // mehce added starts
   /**
   * @brief It is callback function which takes data from "/" topic from Autoware Universe
   */
   void hand_brake_cmd_callback(
     const autoware_auto_vehicle_msgs::msg::HandBrakeCommand::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/" topic from Autoware Universe
   */
   void headlights_cmd_callback(
     const autoware_auto_vehicle_msgs::msg::HeadlightsCommand::ConstSharedPtr msg);
   /**
   * @brief It is callback function which takes data from "/" topic from Autoware Universe
   */
   void raw_control_cmd_callback(
     const autoware_auto_vehicle_msgs::msg::RawControlCommand::ConstSharedPtr msg);
  // mehce added ends
  /**
   * @brief It sends data from interface to low level controller.
   */
  void llc_publisher();
  /**
   * @brief It takes data from interface to low level controller.
   */
  void serial_receive_callback(const char * data, unsigned int len);
  /**
   * @brief It converts the steering angle to steering wheel angle.
   * Steering angle means "Teker açısı" and which is radian.
   * Steering wheel angle means "Direksiyon açısı" and which is degree.
   */
  float steering_tire_to_steering_wheel_angle(float input);
  /**
   * @brief It converts the steering wheel angle to steering angle.
   * Steering angle means "Teker açısı" and which is radian.
   * Steering wheel angle means "Direksiyon açısı" and which is degree.
   */
  float steering_wheel_to_steering_tire_angle(float input);
  /**
   * @brief It converts the gear data which is taken from LLC wrt Autoware Universe messages.
   */
  uint8_t gear_adapter_to_autoware(uint8_t & input);
  /**
   * @brief It converts the gear data which is taken from autoware universe wrt LLC messages.
   */
  void gear_adapter_to_llc(const uint8_t & input);
  /**
   * @brief It converts the headlight data which is taken from LLC wrt Autoware Universe
   * messages.
   */
  uint8_t headlight_adapter_to_autoware(uint8_t & input);
  /**
   * @brief It converts the control mode data which is taken from LLC wrt Autoware Universe
   * messages.
   */
  uint8_t control_mode_adapter_to_autoware(uint8_t & input);
  /**
   * @brief It converts the control mode data which is taken from autoware universe wrt LLC
   * messages.
   */
  void control_mode_adapter_to_llc();
  /**
   * @brief It converts the indicator data which is taken from LLC wrt Autoware Universe messages.
   */
  void indicator_adapter_to_autoware(uint8_t & input);
  /**
   * @brief It converts the indicator data which is taken from autoware universe wrt LLC messages.
   */
  void indicator_adapter_to_llc();
  /**
   * @brief It is the meta converter function that takes all data from LLC and convert them to
   * Autoware messages which are defined as global variable.
   */
  void llc_to_autoware_msg_adapter();
  /**
   * @brief It is the meta converter function that takes all data from Autoware and convert them to
   * LLC Data Structure which are defined as global variable.
   */
  void autoware_to_llc_msg_adapter();
  /**
   * @brief Check the emergency state of autoware
   */
  void onEmergencyState(autoware_auto_system_msgs::msg::EmergencyState::ConstSharedPtr msg);

  void onHazardStatusStamped(
    const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);

  void onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::SharedPtr message);

private:
  std::vector<uint8_t> receive_buffer_;

  /**
   * @brief This function updates PDS system error with latest updates
   */
  void checkPDSSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates BBW system error with latest updates
   */
  void checkBBWSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates EPAS system error with latest updates
   */
  void checkEPASSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates PC Ignition error with latest updates
   */
  void checkPCIgnitionError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates EPAS power error with latest updates
   */
  void checkEPASPowerError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates SBW power error with latest updates
   */
  void checkSBWPowerError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates BBW power error with latest updates
   */
  void checkBBWPowerError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates PC timeout error with latest updates
   */
  void checkPCTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates BCU timeout error with latest updates
   */
  void checkBCUTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates PDS timeout error with latest updates
   */
  void checkPDSTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates EPAS timeout error with latest updates
   */
  void checkEPASTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates BBW timeout error with latest updates
   */
  void checkBBWTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /* input values */

  // From Autoware

  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
  tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr emergency_cmd_ptr;
  tier4_control_msgs::msg::GateMode::ConstSharedPtr gate_mode_cmd_ptr;
  //mehce added starts
  autoware_auto_vehicle_msgs::msg::HandBrakeCommand::ConstSharedPtr hand_brake_cmd_ptr;
  autoware_auto_vehicle_msgs::msg::HeadlightsCommand::ConstSharedPtr head_lights_cmd_ptr;
  autoware_auto_vehicle_msgs::msg::RawControlCommand::ConstSharedPtr raw_control_cmd_ptr;
  //mehce added ends
  bool engage_cmd_{0};

  /* Variables */
  rclcpp::Time control_command_received_time_;
  autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr hazard_status_stamped_;

  // Current state of vehicle (Got from LLC)

  vehicle_current_state_ current_state;
  std_msgs::msg::String error_str;

  // To LLC

  CompToLlcData_ send_data;
  bool is_emergency_{false};
  bool prev_emergency{false};
  float current_emergency_acceleration{0.0};
  bool take_over_requested_{false};

  /* subscribers */
  // From Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_lights_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;
  rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::ConstSharedPtr gate_mode_sub_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::EmergencyState>::SharedPtr
    emergency_state_sub_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::HazardStatusStamped>::SharedPtr
    sub_hazard_status_stamped_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::ConstSharedPtr autoware_state_sub_;
  // mehce added starts
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>::SharedPtr hand_brake_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HeadlightsCommand>::SharedPtr headlights_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::RawControlCommand>::SharedPtr
    raw_control_cmd_sub_;
  // mehce added ends

  /* publishers */
  // To Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_status_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::SteeringWheelStatusStamped>::SharedPtr
    steering_wheel_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr llc_error_pub_;
  // mehce added starts
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HandBrakeReport>::SharedPtr hand_brake_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HeadlightsReport>::SharedPtr headlights_pub_;
  // mehce added ends

  // Timer
  rclcpp::TimerBase::SharedPtr tim_data_sender_;

  /* ros params */
  vehicle_info_util::VehicleInfo vehicle_info_;

  std::string base_frame_id_;
  double loop_rate_{};                    // [Hz]
  float wheel_base_{};                    // [m]
  double command_timeout_ms_{};           // vehicle_cmd timeout [ms]
  bool reverse_gear_enabled_{false};      // reverse gear enabled or not
  float emergency_stop_acceleration{};    // [m/s^2]
  float gear_shift_velocity_threshold{};  // [m/s]
  float max_steering_wheel_angle{};       // [degree]
  float min_steering_wheel_angle{};       // [degree]
  float max_steering_wheel_angle_rate{};  // [degree/sec]
  bool check_steering_angle_rate{};
  float soft_stop_acceleration{};                 // [m/s^2]
  float add_emergency_acceleration_per_second{};  // [m/s^3]
  bool enable_emergency{};
  bool enable_cmd_timeout_emergency{};
  bool enable_debugger{};
  float steering_offset{0.0};

  // Diagnostic Updater Object
  diagnostic_updater::Updater updater_;

  // Define a struct to store autonomous system faults
  struct SystemError
  {
    bool pds_system_error = false;
    bool bbw_system_error = false;
    bool epas_system_error = false;
    bool pc_ignition_error = false;
    bool epas_pwr_error = false;
    bool sbw_pwr_error = false;
    bool bbw_pwr_error = false;
    bool pc_timeout = false;
    bool bcu_timeout = false;
    bool pds_timeout = false;
    bool epas_timeout = false;
    bool bbw_timeout = false;
  };

  SystemError system_error_diagnostics_;
  std::vector<float> wheel_angle_{-700.0, -650.0, -600.0, -550.0, -500.0, -450.0, -350.0, -250.0,
                                  -150.0, -50.0,  -25.0,  -10.0,  10.0,   25.0,   50.0,   150.0,
                                  250.0,  350.0,  450.0,  500.0,  550.0,  600.0,  650.0,  700.0};
  std::vector<float> steering_angle_{
    -0.64993461705671285,  -0.61082144199407651,  -0.57803454274344535,  -0.54093534890626638,
    -0.49747436744167056,  -0.44171141830811872,  -0.35975359145439723,  -0.25783474728117478,
    -0.15425576854889872,  -0.043130533956498907, -0.015611978147209488, -0.009269611330923487,
    0.0088624445735869754, 0.032612469460628908,  0.059706118376520081,  0.16757600370090606,
    0.2716162219588133,    0.37091961814500307,   0.46752682569741805,   0.51357256281933916,
    0.55710949337717441,   0.61005324248865378,   0.63768293444784307,   0.64};

  //can testing codes

  can_msgs::msg::Frame::SharedPtr msg_recv_can_frame_;
  can_msgs::msg::Frame msg_send_can_frame_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_recv_frame_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_send_frame_;

  void receivedFrameCallback(can_msgs::msg::Frame::SharedPtr msg);
  void sendCanFrame();
  void mechanical_error_check();
  void electrical_error_check();

  LlcToCompData llc_to_comp_data_ {};
  CompToLlcCmd comp_to_llc_cmd {};
};
#endif  // LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_
