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

#include <leo_vcu_driver/interface_plugins/leo_vcu_can_interface.hpp>

namespace leo_vcu_driver::can_interface
{

using namespace std::placeholders;

void CanInterface::initialize(rclcpp::Node * node)
{
  node_ = node;

  can_frame_sub_ = node_->create_subscription<can_msgs::msg::Frame>(
    "/from_can_bus", rclcpp::QoS{1}, std::bind(&CanInterface::can_receive_callback, this, _1));

  can_frame_pub_ =node_->create_publisher<can_msgs::msg::Frame>("/to_can_bus", rclcpp::QoS{10});
}

bool CanInterface::update_received_frame(
  leo_vcu_driver::vehicle_interface::LlcToCompData & llc_to_comp_data)
{
  if(can_frame_sub_->get_publisher_count() < 1)
  {
    RCLCPP_WARN(
      node_->get_logger(), "WAITING FOR CAN TOPIC SUBSCRIPTION!");
    return false;
  }
  llc_to_comp_data = llc_to_comp_data_;
  return true;
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
  header.frame_id = node_->get_parameter("base_frame_id").get_parameter_value().get<std::string>();
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

void CanInterface::can_receive_callback(can_msgs::msg::Frame::SharedPtr msg)
{
  switch (msg->id) {
    case 1041: // get linear vehicle velocity and front wheel angle
    {
      std::memcpy(&llc_to_comp_data_.vehicle_dyn_info, &msg->data,
                  sizeof(llc_to_comp_data_.vehicle_dyn_info));
    }
    break;
    case 1042: // fuel, blinker, headlight, wiper, gear, mode, hand_brake and horn publisher
    {
      std::memcpy(&llc_to_comp_data_.vehicle_sgl_status, &msg->data,
                  sizeof(llc_to_comp_data_.vehicle_sgl_status));

    }
    break;
    case 1043: // intervention, ready, motion_allow, throttle, brake, front_steer
    {
      std::memcpy(&llc_to_comp_data_.motion_info, &msg->data,
                  sizeof(llc_to_comp_data_.motion_info));
    }
    break;
    case 1044: // temperature and rpm
    {
      std::memcpy(&llc_to_comp_data_.motor_info, &msg->data,
                  sizeof(llc_to_comp_data_.motor_info));
    }
    break;
    case 1045: // errors
    {
      std::memcpy(&llc_to_comp_data_.error_info, &msg->data,
                  sizeof(llc_to_comp_data_.error_info));
    }
    break;
    case 1024: case 1025: case 1026: case 1027: // sending frame ids
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Invalid CanId: %d", msg->id);
      break;
  }
}

}  // namespace leo_vcu_driver::can_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(leo_vcu_driver::can_interface::CanInterface,
                       leo_vcu_driver::LeoVcuDriverPlugin)
