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
//
// Co-developed by Tier IV, Inc. Apex.AI, Inc. and Leo Drive Teknoloji A.Ş.

/// \copyright Copyright 2022 Leo Drive Teknoloji A.Ş.
/// \file
/// \brief This file defines the leo_vcu_driver class.

#ifndef LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_
#define LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <pluginlib/class_loader.hpp>
#include <leo_vcu_driver/interface_plugins/leo_vcu_driver_plugin.hpp>

#include <leo_vcu_driver/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <leo_vcu_driver/vehicle_interface.h>
#include <linux/can.h>
#include <bitset>
#include <string>
#include <vector>

class LeoVcuDriver : public rclcpp::Node
{
public:
  LeoVcuDriver();
  ~LeoVcuDriver() override { }

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
  /**
   * @brief It is callback function which takes data from "/system/emergency/emergency_state" topic
   * from Autoware Universe.
   */
  void onEmergencyState(autoware_auto_system_msgs::msg::EmergencyState::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/system/emergency/hazard_status" topic
   * from Autoware Universe.
   */
  void onHazardStatusStamped(
    const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);
  /**
   * @brief It is callback function which takes data from "/autoware/state" topic from
   * Autoware Universe.
   */
  void onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::SharedPtr message);
  /**
   * @brief It is callback function which takes data from "/control/command/actuation_cmd" topic in
   * Autoware Universe.
   */
  void actuator_cmd_callback(
    const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr msg);

  //Hand Brake, headlights and raw control commands are not published yet by Autoware.Universe
  /**
   * @brief It is callback function which takes data from "/" topic from Autoware Universe

   void hand_brake_cmd_callback(
     const autoware_auto_vehicle_msgs::msg::HandBrakeCommand::ConstSharedPtr msg);
  */
  /**
   * @brief It is callback function which takes data from "/" topic from Autoware Universe

   void headlights_cmd_callback(
     const autoware_auto_vehicle_msgs::msg::HeadlightsCommand::ConstSharedPtr msg);
  */
  /**
   * @brief It is callback function which takes data from "/" topic from Autoware Universe

   void raw_control_cmd_callback(
     const autoware_auto_vehicle_msgs::msg::RawControlCommand::ConstSharedPtr msg);
  */
  /**
   * @brief It sends data from interface to low level controller.
   */
  void llc_interface_adapter();
  /**
   * @brief It publishes current state of vehicle to Autoware.Universe
   */
  void publish_current_vehicle_state();
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
   * @brief It is the meta converter function that takes all data from LLC and convert them to
   * leo_vcu_msgs which are ROS2 msg type.
   */
  void llc_to_state_report_msg_adapter();
private:
  /**
   * @brief This function updates Motor Running system error with latest updates
   */
  void checkMotorRunningError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates Kl75 error with latest updates
   */
  void checkKl75Error(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates PDS timeout error with latest updates
   */
  void checkPDSTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates PDS bus error with latest updates
   */
  void checkPDSBusError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates BY Wire system error with latest updates
   */
  void checkByWireError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates EPAS power error with latest updates
   */
  void checkEPASPowerError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates Brake power error with latest updates
   */
  void checkBrakePowerError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates Throttle-ecu timeout error with latest updates
   */
  void checkThrottleTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates G29 timeout error with latest updates
   */
  void checkG29TimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates EPAS system error with latest updates
   */
  void checkEPASSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates EPAS timeout error with latest updates
   */
  void checkEPASTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates Brake System error with latest updates
   */
  void checkBrakeSystemError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates Brake timeout error with latest updates
   */
  void checkBrakeTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);
  /**
   * @brief This function updates PC timeout error with latest updates
   */
  void checkPCTimeoutError(diagnostic_updater::DiagnosticStatusWrapper & stat);

  pluginlib::ClassLoader<leo_vcu_driver::LeoVcuDriverPlugin> plugin_loader_;
  std::shared_ptr<leo_vcu_driver::LeoVcuDriverPlugin> driver_interface_plugin_;

  /* input values */

  // From Autoware
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
  tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr emergency_cmd_ptr;
  tier4_control_msgs::msg::GateMode::ConstSharedPtr gate_mode_cmd_ptr;
  tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr actuation_cmd_ptr;
  leo_vcu_msgs::msg::StateReport vehicle_state_report_msg_;

  /*
  autoware_auto_vehicle_msgs::msg::HandBrakeCommand::ConstSharedPtr hand_brake_cmd_ptr;
  autoware_auto_vehicle_msgs::msg::HeadlightsCommand::ConstSharedPtr head_lights_cmd_ptr;
  autoware_auto_vehicle_msgs::msg::RawControlCommand::ConstSharedPtr raw_control_cmd_ptr;
   */

  bool engage_cmd_{0};

  /* Variables */
  rclcpp::Time control_command_received_time_;
  autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr hazard_status_stamped_;

  // Current state of vehicle (Got from LLC)
  leo_vcu_driver::vehicle_interface::vehicle_current_state_ current_state;
  std_msgs::msg::String error_str;
  leo_vcu_driver::vehicle_interface::LlcToCompData llc_to_comp_data_ {};

  // CAN interface msg (Got from LLC)
  can_msgs::msg::Frame::SharedPtr received_can_frame_msg_;

  // To LLC
  leo_vcu_driver::vehicle_interface::CompToLlcCmd comp_to_llc_cmd {};

  bool is_emergency_{false};
  bool prev_emergency_{false};
  float current_emergency_acceleration_{0.0};
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
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::ConstSharedPtr
    autoware_state_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::ConstSharedPtr
    actuation_cmd_sub_;
  /*
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HandBrakeCommand>::SharedPtr
   hand_brake_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HeadlightsCommand>::SharedPtr
   headlights_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::RawControlCommand>::SharedPtr
   raw_control_cmd_sub_;
  */
  // From CAN interface
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;


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

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HandBrakeReport>::SharedPtr hand_brake_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HeadlightsReport>::SharedPtr headlights_pub_;
  rclcpp::Publisher<leo_vcu_msgs::msg::StateReport>::SharedPtr vehicle_state_report_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr tim_data_sender_;

  /* ros params */
  vehicle_info_util::VehicleInfo vehicle_info_;

  std::string interface_mod_;
  std::string base_frame_id_;
  double data_send_rate_{};               // [Hz]
  float wheel_base_{};                    // [m]
  double command_timeout_ms_{};           // vehicle_cmd timeout [ms]
  bool reverse_gear_enabled_{false};      // reverse gear enabled or not
  float gear_shift_velocity_threshold{};  // [m/s]
  float max_steering_wheel_angle{};       // [degree]
  float min_steering_wheel_angle{};       // [degree]
  float max_steering_wheel_angle_rate{};  // [degree/sec]
  bool check_steering_angle_rate{};
  bool enable_emergency{};
  bool enable_cmd_timeout_emergency{};
  float steering_offset{0.0};
  float emergency_stop_acceleration{};
  float soft_stop_acceleration{};         // [m/s^2]
  float add_emergency_acceleration_per_second{};
  bool enable_long_actuation_mode{};

  // Diagnostic Updater Object
  diagnostic_updater::Updater updater_;

  // Define a struct to store autonomous system faults
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

  SystemError system_error_diagnostics_;
  /**
   * @brief It checks mechanical errors from llc message. (Motor Running, KL75)
   */
  void mechanical_error_check(SystemError & latest_system_error);
  /**
   * @brief It checks electrical errors from llc message. (EPAS, BBW, DBW etc.)
   */
  void electrical_error_check(SystemError & latest_system_error);

  // TODO(ismet): update w/bayram
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

};
#endif  // LEO_VCU_DRIVER__LEO_VCU_DRIVER_HPP_
