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

// A ROS 2 node that merges two JointCommand topics.
//
// Design goals:
// - Publish at the same rate as the RL command topic (RL topic is the "clock").
// - Respect RL commands as the base, optionally overriding upper-body joints from ROS.
// - Allow switching upper-body source between "ros" and "rl" at runtime.
// - Smooth transitions to avoid sudden command jumps.

#include <algorithm>
#include <cctype>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "interface_protocol/msg/joint_command.hpp"

namespace engineai_ros2
{

using JointCommand = interface_protocol::msg::JointCommand;
using SetBool = std_srvs::srv::SetBool;

class JointCommandMuxNode final : public rclcpp::Node
{
public:
  JointCommandMuxNode()
  : rclcpp::Node("joint_command_mux")
  {
    // ----------------
    // Parameters
    // ----------------
    // Which source should provide upper-body joints.
    // - "ros": override upper-body joints from topic_ros
    // - "rl": do not override (RL is used for all joints)
    this->declare_parameter<std::string>("upper_source", "ros");

    // Duration (seconds) used to blend from the previous published output to the new mode.
    this->declare_parameter<double>("transition_time", 0.3);

    // For safety, blending only position is usually best. If set false, all fields are blended.
    this->declare_parameter<bool>("blend_position_only", false);

    // Joint names in the same order as JointCommand arrays.
    // This avoids relying on URDF parsing order.
    this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});

    // Keywords used to classify lower-body joints. If a joint name contains any keyword
    // (case-insensitive), it is treated as lower-body and never overridden from ROS.
    this->declare_parameter<std::vector<std::string>>(
      "lower_body_keywords", std::vector<std::string>{"HIP", "KNEE", "ANKL"});

    // Topics
    this->declare_parameter<std::string>("topic_out", "/hardware/joint_command");
    this->declare_parameter<std::string>("topic_ros", "/hardware/joint_command_ros");
    this->declare_parameter<std::string>("topic_rl", "/hardware/joint_command_rl");

    upper_source_ = this->get_parameter("upper_source").as_string();
    transition_time_ = this->get_parameter("transition_time").as_double();
    blend_position_only_ = this->get_parameter("blend_position_only").as_bool();
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    lower_body_keywords_ = this->get_parameter("lower_body_keywords").as_string_array();

    topic_out_ = this->get_parameter("topic_out").as_string();
    topic_ros_ = this->get_parameter("topic_ros").as_string();
    topic_rl_ = this->get_parameter("topic_rl").as_string();

    build_upper_body_indices();

    // ----------------
    // QoS
    // ----------------
    // Use SensorDataQoS for subscriptions to avoid QoS incompatibilities.
    const auto qos_sub = rclcpp::SensorDataQoS();
    const auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // ----------------
    // Pub/Sub
    // ----------------
    publisher_ = this->create_publisher<JointCommand>(topic_out_, qos_pub);

    sub_ros_ = this->create_subscription<JointCommand>(
      topic_ros_, qos_sub,
      std::bind(&JointCommandMuxNode::on_ros_command, this, std::placeholders::_1));

    // RL is the clock: publish happens only from this callback.
    sub_rl_ = this->create_subscription<JointCommand>(
      topic_rl_, qos_sub,
      std::bind(&JointCommandMuxNode::on_rl_command, this, std::placeholders::_1));

    // ----------------
    // Services
    // ----------------
    // data=true  -> upper_source="rl"
    // data=false -> upper_source="ros"
    srv_set_upper_rl_ = this->create_service<SetBool>(
      "~/set_upper_rl",
      std::bind(
        &JointCommandMuxNode::on_set_upper_rl,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    // ----------------
    // Runtime parameter update
    // ----------------
    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&JointCommandMuxNode::on_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Started joint_command_mux (upper_source=%s, transition_time=%.3f, blend_position_only=%s).",
      upper_source_.c_str(),
      transition_time_,
      blend_position_only_ ? "true" : "false");
  }

private:
  // dst = (1-alpha) * dst + alpha * src
  template <typename VecT>
  static void copy_or_blend(VecT & dst, const VecT & src, size_t idx, double alpha)
  {
    if (dst.size() <= idx || src.size() <= idx) {
      return;
    }
    if (alpha <= 0.0) {
      dst[idx] = src[idx];
      return;
    }
    dst[idx] = (1.0 - alpha) * dst[idx] + alpha * src[idx];
  }

  static std::string to_upper(std::string s)
  {
    for (auto & c : s) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return s;
  }

  bool is_lower_body_joint(const std::string & joint_name) const
  {
    const auto upper = to_upper(joint_name);
    for (const auto & k : lower_body_keywords_) {
      if (!k.empty() && upper.find(to_upper(k)) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  void build_upper_body_indices()
  {
    upper_body_indices_.clear();

    if (joint_names_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'joint_names' is empty. No joints will be overridden from ROS.");
      return;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!is_lower_body_joint(joint_names_[i])) {
        upper_body_indices_.push_back(i);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu joints. Upper-body indices=%zu.",
      joint_names_.size(),
      upper_body_indices_.size());
  }

  // Blend factor: 0 -> 1
  double compute_alpha() const
  {
    if (!transition_active_ || transition_time_ <= 0.0) {
      return 1.0;
    }
    const double dt = (this->now() - transition_start_).seconds();
    return std::clamp(dt / transition_time_, 0.0, 1.0);
  }

  void start_transition_from_last_publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    transition_from_ = last_published_;
    if (transition_from_ && transition_time_ > 0.0) {
      transition_active_ = true;
      transition_start_ = this->now();
    } else {
      transition_active_ = false;
    }
  }

  void set_upper_source(const std::string & source)
  {
    if (source != "ros" && source != "rl") {
      RCLCPP_WARN(get_logger(), "Invalid upper_source='%s'.", source.c_str());
      return;
    }
    if (upper_source_ == source) {
      return;
    }
    start_transition_from_last_publish();
    upper_source_ = source;
    RCLCPP_INFO(get_logger(), "upper_source switched to '%s'.", upper_source_.c_str());
  }

  // ---------- callbacks ----------
  void on_ros_command(const JointCommand::SharedPtr msg)
  {
    bool was_null = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      was_null = !latest_ros_;
      latest_ros_ = msg;
    }

    // If we are already in ROS-upper mode and this is the first ROS message, blend into it.
    if (upper_source_ == "ros" && was_null) {
      start_transition_from_last_publish();
    }
  }

  void on_rl_command(const JointCommand::SharedPtr rl_msg)
  {
    // 1) Build target output from RL.
    auto out_msg = std::make_unique<JointCommand>(*rl_msg);
    out_msg->header.stamp = this->now();

    JointCommand::SharedPtr ros_msg;
    std::shared_ptr<JointCommand> from_msg;
    bool do_transition = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ros_msg = latest_ros_;
      from_msg = transition_from_;
      do_transition = transition_active_ && static_cast<bool>(from_msg);
    }

    // 2) Apply upper-body override from ROS if enabled.
    if (upper_source_ == "ros" && ros_msg) {
      for (const size_t idx : upper_body_indices_) {
	RCLCPP_INFO(get_logger(), "idx %ld ros:%7.3f rl:%7.3f", idx, ros_msg->position[idx], out_msg->position[idx]);
        copy_or_blend(out_msg->position, ros_msg->position, idx, 0.0);
        if (!blend_position_only_) {
          copy_or_blend(out_msg->velocity, ros_msg->velocity, idx, 0.0);
          copy_or_blend(out_msg->feed_forward_torque, ros_msg->feed_forward_torque, idx, 0.0);
          copy_or_blend(out_msg->torque, ros_msg->torque, idx, 0.0);
          copy_or_blend(out_msg->stiffness, ros_msg->stiffness, idx, 0.0);
          copy_or_blend(out_msg->damping, ros_msg->damping, idx, 0.0);
        }
      }
    }

    // 3) Blend from previous published output to the new target (both directions).
    if (do_transition) {
      const double alpha = compute_alpha();
      const double alpha2 = 1.0 - alpha;  // Pull target towards "from" by (1-alpha).

      for (const size_t idx : upper_body_indices_) {
        // Desired: out = (1-alpha)*from + alpha*target
        // copy_or_blend does: target = (1-a)*target + a*from
        // so we use a = (1-alpha).
        copy_or_blend(out_msg->position, from_msg->position, idx, alpha2);
        if (!blend_position_only_) {
          copy_or_blend(out_msg->velocity, from_msg->velocity, idx, alpha2);
          copy_or_blend(out_msg->feed_forward_torque, from_msg->feed_forward_torque, idx, alpha2);
          copy_or_blend(out_msg->torque, from_msg->torque, idx, alpha2);
          copy_or_blend(out_msg->stiffness, from_msg->stiffness, idx, alpha2);
          copy_or_blend(out_msg->damping, from_msg->damping, idx, alpha2);
        }
      }

      if (alpha >= 1.0) {
        std::lock_guard<std::mutex> lock(mutex_);
        transition_active_ = false;
        transition_from_.reset();
      }
    }

    // 4) Publish and store last published command.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_published_ = std::make_shared<JointCommand>(*out_msg);
    }
    publisher_->publish(std::move(out_msg));
  }

  void on_set_upper_rl(
    const std::shared_ptr<SetBool::Request> req,
    std::shared_ptr<SetBool::Response> res)
  {
    set_upper_source(req->data ? "rl" : "ros");
    res->success = true;
    res->message = std::string("upper_source set to ") + (req->data ? "rl" : "ros");
  }

  rcl_interfaces::msg::SetParametersResult
  on_parameters(const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      if (p.get_name() == "upper_source") {
        set_upper_source(p.as_string());
      } else if (p.get_name() == "transition_time") {
        transition_time_ = p.as_double();
      } else if (p.get_name() == "blend_position_only") {
        blend_position_only_ = p.as_bool();
      } else if (p.get_name() == "joint_names") {
        joint_names_ = p.as_string_array();
        build_upper_body_indices();
      } else if (p.get_name() == "lower_body_keywords") {
        lower_body_keywords_ = p.as_string_array();
        build_upper_body_indices();
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

private:
  // Parameters
  std::string upper_source_{"ros"};
  double transition_time_{0.3};
  bool blend_position_only_{true};
  std::vector<std::string> joint_names_;
  std::vector<std::string> lower_body_keywords_;

  // Topics
  std::string topic_out_;
  std::string topic_ros_;
  std::string topic_rl_;

  // Joint mask
  std::vector<size_t> upper_body_indices_;

  // State (guarded)
  std::mutex mutex_;
  JointCommand::SharedPtr latest_ros_;
  std::shared_ptr<JointCommand> last_published_;
  std::shared_ptr<JointCommand> transition_from_;

  // Transition
  bool transition_active_{false};
  rclcpp::Time transition_start_{0, 0, RCL_ROS_TIME};

  // ROS entities
  rclcpp::Publisher<JointCommand>::SharedPtr publisher_;
  rclcpp::Subscription<JointCommand>::SharedPtr sub_ros_;
  rclcpp::Subscription<JointCommand>::SharedPtr sub_rl_;
  rclcpp::Service<SetBool>::SharedPtr srv_set_upper_rl_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace engineai_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<engineai_ros2::JointCommandMuxNode>());
  rclcpp::shutdown();
  return 0;
}
