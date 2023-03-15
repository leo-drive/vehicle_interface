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

#ifndef CAN_INTERFACE_PLUGINS__CAN_HPP_
#define CAN_INTERFACE_PLUGINS__CAN_HPP_


#include <rclcpp/rclcpp.hpp>
#include <leo_vcu_driver/interface_plugins/leo_vcu_driver_plugin.hpp>
#include <leo_vcu_driver/vehicle_interface.h>

namespace leo_vcu_driver::can_interface
{

class CanInterface : public leo_vcu_driver::LeoVcuDriverPlugin
{
public:
  rclcpp::Node * node_;

  leo_vcu_driver::vehicle_interface::LlcToCompData llc_to_comp_data_ {};

  leo_vcu_driver::vehicle_interface::LlcCanMsg llc_can_msgs;
  can_msgs::msg::Frame::SharedPtr received_can_frame_msg_;

  void initialize(rclcpp::Node * node) override;

  bool update_received_frame(
    leo_vcu_driver::vehicle_interface::LlcToCompData & llc_to_comp_data) override;

  void llc_publisher(
    const leo_vcu_driver::vehicle_interface::CompToLlcCmd & comp_to_llc_cmd) override;

  void can_receive_callback(can_msgs::msg::Frame::SharedPtr msg);

private:
  // To LLC
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;

  // From LLC
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;
};

}  // namespace leo_vcu_driver::can_interface

#endif  // CAN_INTERFACE_PLUGINS__CAN_HPP_
