// Copyright 2026 Kei Okada
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

#ifndef ENGINEAI_HARDWARE_INTERFACE_HPP_
#define ENGINEAI_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace engineai_hardware_interface
{
class EngineAI_SystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EngineAI_SystemPositionOnlyHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the engineai simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  double default_stiffness_{20.0};
  double default_damping_{1.0};
  std::vector<double> stiffness_per_joint_;
  std::vector<double> damping_per_joint_;

  // Publish & Subscribe
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<interface_protocol::msg::ImuInfo>::SharedPtr imu_sub_;
  rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::mutex mtx_;
  std::unordered_map<std::string, double> latest_pos_;
  std::unordered_map<std::string, double> latest_vel_;
  std::unordered_map<std::string, double> latest_tor_;  
  sensor_msgs::msg::Imu latest_imu_;
  std::vector<std::string> joint_names_;
};

}  // namespace engineai_hardware_interface

#endif  // ENGINEAI_HARDWARE_INTERFACE_HPP_
