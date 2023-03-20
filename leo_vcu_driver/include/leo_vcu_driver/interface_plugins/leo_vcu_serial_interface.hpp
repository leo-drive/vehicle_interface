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

#ifndef SERIAL_INTERFACE_PLUGINS__SERIAL_HPP_
#define SERIAL_INTERFACE_PLUGINS__SERIAL_HPP_


#include <rclcpp/rclcpp.hpp>
#include <leo_vcu_driver/interface_plugins/leo_vcu_driver_plugin.hpp>
#include <leo_vcu_driver/vehicle_interface.h>
#include <leo_vcu_driver/async_serial/AsyncSerial.h>
#include <leo_vcu_driver/async_serial/checksum.h>

#include <experimental/optional>

namespace leo_vcu_driver::serial_interface
{

using namespace std::experimental;

class SerialInterface : public leo_vcu_driver::LeoVcuDriverPlugin
{
public:
  rclcpp::Node * node_;

  leo_vcu_driver::vehicle_interface::LlcToCompData llc_to_comp_data_ {};

  CallbackAsyncSerial * serial;

  int serial_counter{0};

  const std::string serial_name_{"/dev/ttyLLC"};
  bool serial_ready{false};

  void initialize(rclcpp::Node * node) override;
  /**
   * @brief It updates driver' can frame according to the Serial Communication received data.
   */
  bool update_received_frame(
    leo_vcu_driver::vehicle_interface::LlcToCompData & llc_to_comp_data) override;
  /**
   * @brief It sends data from driver Serial interface to low level controller.
   */
  void llc_publisher(
    const leo_vcu_driver::vehicle_interface::CompToLlcCmd & comp_to_llc_cmd) override;
  /**
   * @brief It is callback function which takes vehicle data from Serial Connection.
   */
  void serial_receive_callback(const char * data, unsigned int len);
private:
  /**
   * @brief It converts data from LLC to LlcToCompData.
   */
  optional<leo_vcu_driver::vehicle_interface::LlcToCompData> find_llc_to_comp_msg(
    const char * data, unsigned int len);
  /**
   * @brief It packs data for Serial Interface.
   */
  static std::vector<char> pack_serial_data(
    const leo_vcu_driver::vehicle_interface::CompToLlcCmd & data);

  // Serial frame ids
   const uint8_t comp_to_llc_msg_frame_id {16U};
   const uint8_t comp_to_llc_msg_eof_id {17U};

  const uint8_t llc_to_comp_msg_frame_id {99U};
  const uint8_t llc_to_comp_msg_eof_id {100U};

  std::vector<uint8_t> receive_buffer_;
};

}  // namespace leo_vcu_driver::serial_interface

#endif  // SERIAL_INTERFACE_PLUGINS__SERIAL_HPP_
