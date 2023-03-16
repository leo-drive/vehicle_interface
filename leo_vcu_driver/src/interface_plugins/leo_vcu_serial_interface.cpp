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

#include <leo_vcu_driver/interface_plugins/leo_vcu_serial_interface.hpp>

namespace leo_vcu_driver::serial_interface
{

using namespace std::placeholders;

void SerialInterface::initialize(rclcpp::Node * node)
{
  node_ = node;
}

bool SerialInterface::update_received_frame(
  leo_vcu_driver::vehicle_interface::LlcToCompData & llc_to_comp_data)
{
  llc_to_comp_data = llc_to_comp_data_;
}

void SerialInterface::llc_publisher(
  const leo_vcu_driver::vehicle_interface::CompToLlcCmd & comp_to_llc_cmd)
{
  if (!serial->isOpen()) {
    try {
      serial->open(serial_name_, 115200);
      serial->setCallback(bind(
        &SerialInterface::serial_receive_callback, this, _1, _2));
    } catch (boost::system::system_error & e) {
      RCLCPP_WARN(node_->get_logger(), "%s", e.what());
      serial_ready = false;
      return;
    }
  } else {
    serial_ready = true;
  }
}

void SerialInterface::serial_receive_callback(const char * data, unsigned int len)
{
  auto llc_to_comp_data_{find_llc_to_comp_msg(data, len)};
  if (!llc_to_comp_data_) {
    RCLCPP_ERROR(node_->get_logger(), "Received data is not viable!\n");
    return;
  }
}

optional<leo_vcu_driver::vehicle_interface::LlcToCompData> SerialInterface::find_llc_to_comp_msg(
  const char * data, unsigned int len)
{
  receive_buffer_.insert(receive_buffer_.end(), data, data + len);

  // Look data size for checking there is enough data to contain a data structure
  if (receive_buffer_.size() < sizeof(leo_vcu_driver::vehicle_interface::LlcToCompData)) {
    return std::experimental::nullopt;
  }

  size_t search_range =
    receive_buffer_.size() - sizeof(leo_vcu_driver::vehicle_interface::LlcToCompData) + 1;

  for (size_t i = 0; i < search_range; i++) {

    const auto llc_data = reinterpret_cast<leo_vcu_driver::vehicle_interface::LlcToCompData*>(
      receive_buffer_.data() + i);
    if (
      llc_data->frame_id1 == llc_to_comp_msg_frame_id &&
      llc_data->frame_id2 == llc_to_comp_msg_frame_id &&
      llc_data->eof_id1 == llc_to_comp_msg_eof_id && llc_data->eof_id2 == llc_to_comp_msg_eof_id)
    {
      const uint16_t crc = crc_16(receive_buffer_.data() + i, sizeof(leo_vcu_driver::vehicle_interface::LlcToCompData) - 4);
      if (crc == llc_data->crc) {
        leo_vcu_driver::vehicle_interface::LlcToCompData valid_data{*llc_data};
        receive_buffer_.erase(
          receive_buffer_.begin(),
          receive_buffer_.begin() + static_cast<long>(i) + sizeof(leo_vcu_driver::vehicle_interface::LlcToCompData));
        return std::experimental::make_optional(valid_data);
      }
    }
  }
  return std::experimental::nullopt;
}

std::vector<char> SerialInterface::pack_serial_data(
  const leo_vcu_driver::vehicle_interface::CompToLlcCmd & data)
{
  const auto ptr{reinterpret_cast<const uint8_t *>(&data)};
  std::vector<char> dataVec(ptr, ptr + sizeof data);
  return dataVec;
}

}  // namespace leo_vcu_driver::serial_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(leo_vcu_driver::serial_interface::SerialInterface, leo_vcu_driver::LeoVcuDriverPlugin)
