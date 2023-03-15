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

#ifndef LEO_VCU_DRIVER__LEO_VCU_DRIVER_PLUGIN_HPP_
#define LEO_VCU_DRIVER__LEO_VCU_DRIVER_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>

#include <leo_vcu_driver/vehicle_interface.h>


namespace leo_vcu_driver
{

class LeoVcuDriverPlugin
{
public:

  virtual ~LeoVcuDriverPlugin() = default;

  virtual void initialize(rclcpp::Node * node) = 0;

  virtual bool update_received_frame(
    leo_vcu_driver::vehicle_interface::LlcToCompData & llc_to_comp_data) = 0;

  virtual void llc_publisher(
    const leo_vcu_driver::vehicle_interface::CompToLlcCmd & comp_to_llc_cmd) = 0;

};
}  // namespace mission_planner

#endif  // LEO_VCU_DRIVER__LEO_VCU_DRIVER_PLUGIN_HPP_
