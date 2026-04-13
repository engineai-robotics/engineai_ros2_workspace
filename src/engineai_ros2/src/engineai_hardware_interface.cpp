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

#include "engineai_ros2/engineai_hardware_interface.hpp"
#include "engineai_ros2/engineai_util.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "interface_protocol/msg/joint_override_command.hpp"

namespace engineai_hardware_interface
{

hardware_interface::CallbackReturn EngineAI_SystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  RCLCPP_INFO(get_logger(), "Robot hardware_component update_rate is %dHz", info_.rw_rate);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Each joint may have 0, 1, or 2 command interfaces.
    // Leg joints (HIP/KNEE/ANKLE) are state-only; upper-body joints have position + velocity.
    if (joint.command_interfaces.size() > 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces. 0, 1, or 2 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto & ci : joint.command_interfaces) {
      if (ci.name != hardware_interface::HW_IF_POSITION &&
          ci.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has unexpected command interface '%s'. "
          "'%s' or '%s' expected.",
          joint.name.c_str(), ci.name.c_str(),
          hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interfaces. 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has '%s' state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Build joint name lists
  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());
  command_joint_names_.clear();

  for (const auto & j : info_.joints) {
    joint_names_.push_back(j.name);
    if (!j.command_interfaces.empty()) {
      command_joint_names_.push_back(j.name);
    }
  }

  command_joint_set_.insert(command_joint_names_.begin(), command_joint_names_.end());

  // Build joint indices for JointOverrideCommand (extract J{N} number from name)
  command_joint_indices_.clear();
  for (const auto & name : command_joint_names_) {
    // Name format: J{index}_{rest}, e.g. J12_WAIST_YAW → 12
    const int idx = std::stoi(name.substr(1, name.find('_') - 1));
    command_joint_indices_.push_back(static_cast<int32_t>(idx));
  }

  RCLCPP_INFO(
    get_logger(), "Total joints: %zu, upper-body (command) joints: %zu",
    joint_names_.size(), command_joint_names_.size());

  const size_t n = joint_names_.size();

  // Initialize per-joint stiffness/damping with defaults
  stiffness_per_joint_.assign(n, default_stiffness_);
  damping_per_joint_.assign(n, default_damping_);

  const auto stiff_map = parse_name_value_csv(
    get_string_param(info_.hardware_parameters, "stiffness_by_joint", ""));
  const auto damp_map  = parse_name_value_csv(
    get_string_param(info_.hardware_parameters, "damping_by_joint", ""));

  for (size_t i = 0; i < n; ++i) {
    const auto & name = joint_names_[i];
    auto itS = stiff_map.find(name);
    if (itS != stiff_map.end()) stiffness_per_joint_[i] = itS->second;
    auto itD = damp_map.find(name);
    if (itD != damp_map.end()) damping_per_joint_[i] = itD->second;
  }

  RCLCPP_INFO(get_logger(),
    "Loaded per-joint stiffness/damping (n=%zu). Defaults: k=%.2f d=%.2f",
    n, default_stiffness_, default_damping_);

  // Build stiffness/damping subsets for command joints only
  command_stiffness_.clear();
  command_damping_.clear();
  for (size_t i = 0; i < n; ++i) {
    if (command_joint_set_.count(joint_names_[i])) {
      command_stiffness_.push_back(stiffness_per_joint_[i]);
      command_damping_.push_back(damping_per_joint_[i]);
    }
  }

  // Publishers
  node_ = std::make_shared<rclcpp::Node>("engineai_hardware_interface");
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(10));
  joint_pub_ = node_->create_publisher<interface_protocol::msg::JointOverrideCommand>(
    "/motion/joint_override_command", rclcpp::QoS(10));

  // Subscribers — use SensorDataQoS (BEST_EFFORT) to match the EngineAI hardware publisher
  joint_sub_ = node_->create_subscription<interface_protocol::msg::JointState>(
    "/hardware/joint_state", rclcpp::SensorDataQoS(),
    [this](interface_protocol::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const auto noj = std::min(joint_names_.size(), msg->position.size());
      for (size_t i = 0; i < noj; ++i) {
        latest_pos_[joint_names_[i]] = msg->position[i];
        latest_vel_[joint_names_[i]] = msg->velocity[i];
        latest_tor_[joint_names_[i]] = msg->torque[i];
      }
    });

  imu_sub_ = node_->create_subscription<interface_protocol::msg::ImuInfo>(
    "/hardware/imu_info", rclcpp::SensorDataQoS(),
    [this](interface_protocol::msg::ImuInfo::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_imu_.header.stamp = node_->now();
      latest_imu_.header.frame_id = "LINK_BASE";
      latest_imu_.orientation.x = msg->quaternion.x;
      latest_imu_.orientation.y = msg->quaternion.y;
      latest_imu_.orientation.z = msg->quaternion.z;
      latest_imu_.orientation.w = msg->quaternion.w;
      latest_imu_.linear_acceleration.x = msg->linear_acceleration.x;
      latest_imu_.linear_acceleration.y = msg->linear_acceleration.y;
      latest_imu_.linear_acceleration.z = msg->linear_acceleration.z;
      latest_imu_.angular_velocity.x = msg->angular_velocity.x;
      latest_imu_.angular_velocity.y = msg->angular_velocity.y;
      latest_imu_.angular_velocity.z = msg->angular_velocity.z;
      latest_imu_.orientation_covariance[0] = 1e-3;
      latest_imu_.orientation_covariance[4] = 1e-3;
      latest_imu_.orientation_covariance[8] = 1e-3;
      latest_imu_.angular_velocity_covariance[0] = 1e-3;
      latest_imu_.angular_velocity_covariance[4] = 1e-3;
      latest_imu_.angular_velocity_covariance[8] = 1e-3;
      latest_imu_.linear_acceleration_covariance[0] = 1e-2;
      latest_imu_.linear_acceleration_covariance[4] = 1e-2;
      latest_imu_.linear_acceleration_covariance[8] = 1e-2;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "publish latest imu: %7.3f %7.3f %7.3f",
        latest_imu_.linear_acceleration.x,
        latest_imu_.linear_acceleration.y,
        latest_imu_.linear_acceleration.z);
      imu_pub_->publish(latest_imu_);
    });

  RCLCPP_INFO(node_->get_logger(), "EngineAI topic-bridge hardware initialized");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EngineAI_SystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  for (int i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  for (const auto & [name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EngineAI_SystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  for (int i = 0; i < hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // Initialize position command interfaces to current state to avoid jumps.
  // Velocity command interfaces have no matching state interface; start at 0.
  for (const auto & [name, descr] : joint_command_interfaces_) {
    if (name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
      set_command(name, 0.0);
    } else {
      set_command(name, get_state(name));
    }
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EngineAI_SystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  for (int i = 0; i < hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EngineAI_SystemPositionOnlyHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Check whether any position command interface is being started or stopped.
  auto has_position = [](const std::vector<std::string> & ifaces) {
    return std::any_of(ifaces.begin(), ifaces.end(), [](const std::string & s) {
      return s.find(hardware_interface::HW_IF_POSITION) != std::string::npos;
    });
  };

  if (has_position(start_interfaces)) {
    controller_active_ = true;
    // Keep current weight_ so the ramp continues smoothly if partially activated.
    RCLCPP_INFO(get_logger(), "Upper-body controller activated; ramping weight %.2f → 1.0", weight_);
  } else if (has_position(stop_interfaces)) {
    controller_active_ = false;
    RCLCPP_INFO(get_logger(), "Upper-body controller deactivated; ramping weight %.2f → 0.0", weight_);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EngineAI_SystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);

  std::lock_guard<std::mutex> lk(mtx_);
  for (const auto & joint : joint_names_) {
    auto it = latest_pos_.find(joint);
    if (it != latest_pos_.end()) {
      set_state(joint + "/position", it->second);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EngineAI_SystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Update weight ramp (0 → 1 when controller activates, 1 → 0 when deactivates).
  const double dt = period.seconds();
  if (controller_active_) {
    weight_ = std::min(1.0, weight_ + ramp_rate_ * dt);
  } else {
    weight_ = std::max(0.0, weight_ - ramp_rate_ * dt);
  }

  // Log command joint positions periodically
  const auto header = make_joint_header_line(command_joint_names_);
  const auto pos_line = make_joint_value_line(command_joint_names_,
    [this](const std::string & key) { return get_command(key); },
    "/position");
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "weight=%.3f\n%s\n%s",
    weight_, header.c_str(), pos_line.c_str());

  // Build JointOverrideCommand for upper-body joints only.
  // weight is sent to the EngineAI onboard controller which handles blending
  // with the RL locomotion controller internally.
  interface_protocol::msg::JointOverrideCommand cmd;
  cmd.header.stamp = get_clock()->now();
  cmd.header.frame_id = "";
  cmd.weight = weight_;
  cmd.joint_indices = command_joint_indices_;

  const size_t nc = command_joint_names_.size();
  cmd.position.reserve(nc);
  cmd.velocity.reserve(nc);
  cmd.feed_forward_torque.assign(nc, 0.0);
  cmd.torque.assign(nc, 0.0);
  cmd.stiffness = command_stiffness_;
  cmd.damping = command_damping_;

  for (const auto & name : command_joint_names_) {
    cmd.position.push_back(get_command(name + "/position"));
    // Feed forward JTC-interpolated velocity so the onboard PD can track smoothly.
    // τ = k*(q_cmd - q) + d*(dq_cmd - dq)  →  d-term ≈ 0 during motion → no "gah"
    cmd.velocity.push_back(get_command(name + "/velocity"));
  }

  joint_pub_->publish(cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace engineai_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  engineai_hardware_interface::EngineAI_SystemPositionOnlyHardware, hardware_interface::SystemInterface)
