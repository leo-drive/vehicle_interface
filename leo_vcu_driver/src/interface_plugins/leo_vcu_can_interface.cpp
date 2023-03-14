// Copyright 2023 Leo Drive Teknoloji A.Ş.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <leo_vcu_driver/leo_vcu_can_interface.hpp>

namespace leo_vcu_driver::can_interface
{

void CanInterface::initialize(rclcpp::Node * node)
{
  node_ = node;
  can_frame_pub_ =node_->create_publisher<can_msgs::msg::Frame>("/to_can_bus", rclcpp::QoS{10});
}
void CanInterface::llc_publisher(
  const leo_vcu_driver::vehicle_interface::CompToLlcCmd & comp_to_llc_cmd)
{
  if(can_frame_pub_->get_subscription_count() < 1)
  {
    RCLCPP_WARN(
      node_->get_logger(), "CAN RECEIVER IS NOT READY!");
    return;
  }

  std::memcpy(&llc_can_msgs.long_cmd_msg.data, &comp_to_llc_cmd.long_cmd, 8);
  std::memcpy(&llc_can_msgs.long_actuation_cmd_msg.data, &comp_to_llc_cmd.long_cmd_actuation, 8);
  std::memcpy(&llc_can_msgs.msg_veh_signal_cmd_frame.data, &comp_to_llc_cmd.vehicle_signal_cmd, 8);
  std::memcpy(
    &llc_can_msgs.msg_front_wheel_cmd_frame.data, &comp_to_llc_cmd.front_wheel_cmd, 8);

  std_msgs::msg::Header header;
  header.frame_id = node_->get_parameter("base_frame_id").get_parameter_value().get<std::string>();;
  header.stamp = node_->get_clock()->now();

  llc_can_msgs.long_cmd_msg.header = header;
  llc_can_msgs.long_cmd_msg.dlc = static_cast<uint8_t>(8);
  llc_can_msgs.long_cmd_msg.id = static_cast<uint32_t>(1024);

  llc_can_msgs.long_actuation_cmd_msg.header = header;
  llc_can_msgs.long_actuation_cmd_msg.dlc = static_cast<uint8_t>(8);
  llc_can_msgs.long_actuation_cmd_msg.id = static_cast<uint32_t>(1025);

  llc_can_msgs.msg_veh_signal_cmd_frame.header = header;
  llc_can_msgs.msg_veh_signal_cmd_frame.dlc = static_cast<uint8_t>(8);
  llc_can_msgs.msg_veh_signal_cmd_frame.id = static_cast<uint32_t>(1026);

  llc_can_msgs.msg_front_wheel_cmd_frame.header = header;
  llc_can_msgs.msg_front_wheel_cmd_frame.dlc = static_cast<uint8_t>(8);
  llc_can_msgs.msg_front_wheel_cmd_frame.id = static_cast<uint32_t>(1027);

  can_frame_pub_->publish(llc_can_msgs.long_cmd_msg);
  can_frame_pub_->publish(llc_can_msgs.long_actuation_cmd_msg);
  can_frame_pub_->publish(llc_can_msgs.msg_veh_signal_cmd_frame);
  can_frame_pub_->publish(llc_can_msgs.msg_front_wheel_cmd_frame);
}

}  // namespace leo_vcu_driver::can_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(leo_vcu_driver::can_interface::CanInterface, leo_vcu_driver::LeoVcuDriverPlugin)
