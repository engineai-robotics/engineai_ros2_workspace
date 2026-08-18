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

// A small utility node that converts geometry_msgs/Twist commands to
// interface_protocol/GamepadKeys for the EngineAI hardware interface.
//
// Notes:
// - The node re-publishes at a fixed rate (timer) using the latest cmd_vel.
// - On SIGINT/SIGTERM/SIGSEGV and on exceptions, it publishes a STOP command.

#include <algorithm>
#include <csignal>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "interface_protocol/msg/gamepad_keys.hpp"

namespace engineai_ros2
{

using GamepadKeys = interface_protocol::msg::GamepadKeys;

class EngineAICmdVelControllerNode final : public rclcpp::Node
{
public:
  EngineAICmdVelControllerNode()
  : rclcpp::Node("engineai_cmd_vel_controller")
  {
    // Parameters
    this->declare_parameter<std::string>("topic_cmd_vel", "cmd_vel");
    this->declare_parameter<std::string>("topic_out", "hardware/gamepad_keys");
    this->declare_parameter<double>("publish_rate_hz", 2.0);
    this->declare_parameter<double>("max_linear", 1.0);
    this->declare_parameter<double>("max_angular", 0.2);

    topic_cmd_vel_ = this->get_parameter("topic_cmd_vel").as_string();
    topic_out_ = this->get_parameter("topic_out").as_string();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
    max_linear_ = this->get_parameter("max_linear").as_double();
    max_angular_ = this->get_parameter("max_angular").as_double();

    // Initialize with zero twist.
    latest_cmd_vel_ = geometry_msgs::msg::Twist();

    // Pub/Sub
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      topic_cmd_vel_,
      rclcpp::QoS(10),
      std::bind(&EngineAICmdVelControllerNode::on_cmd_vel, this, std::placeholders::_1));

    gamepad_pub_ = this->create_publisher<GamepadKeys>(topic_out_, rclcpp::QoS(10));

    // Timer
    const double safe_hz = (publish_rate_hz_ > 0.0) ? publish_rate_hz_ : 2.0;
    const auto period = std::chrono::duration<double>(1.0 / safe_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EngineAICmdVelControllerNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Started engineai_cmd_vel_controller (cmd_vel='%s' -> '%s', rate=%.2f Hz).",
      topic_cmd_vel_.c_str(),
      topic_out_.c_str(),
      safe_hz);
  }

  void publish_stop()
  {
    publish_from_twist(geometry_msgs::msg::Twist());
  }

private:
  static double clamp_abs(double v, double limit)
  {
    if (limit <= 0.0) {
      return 0.0;
    }
    return std::clamp(v, -limit, limit);
  }

  void publish_from_twist(const geometry_msgs::msg::Twist & cmd)
  {
    GamepadKeys msg;

    // Analog input range is typically [-1, 1].
    // Here we scale by max_* parameters.
    const double lx = clamp_abs(cmd.linear.x / max_linear_, 1.0);
    const double ly = clamp_abs(cmd.linear.y / max_linear_, 1.0);
    const double ry = clamp_abs(cmd.angular.z / max_angular_, 1.0);

    msg.analog_states[GamepadKeys::LEFT_STICK_X] = lx;
    msg.analog_states[GamepadKeys::LEFT_STICK_Y] = ly;
    msg.analog_states[GamepadKeys::RIGHT_STICK_Y] = ry;

    // Throttle logs to keep console readable.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "publish gamepad_keys: lx=%.2f ly=%.2f ry=%.2f", lx, ly, ry);

    gamepad_pub_->publish(msg);
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_vel_ = *msg;
  }

  void on_timer()
  {
    publish_from_twist(latest_cmd_vel_);
  }

private:
  std::string topic_cmd_vel_;
  std::string topic_out_;
  double publish_rate_hz_{2.0};
  double max_linear_{1.0};
  double max_angular_{0.2};

  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<GamepadKeys>::SharedPtr gamepad_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

static std::weak_ptr<EngineAICmdVelControllerNode> g_node_weak;

static void signal_handler(int sig)
{
  if (auto node = g_node_weak.lock()) {
    RCLCPP_ERROR(node->get_logger(), "Caught signal %d, publishing STOP then shutdown", sig);
    node->publish_stop();
  }
  rclcpp::shutdown();
}

}  // namespace engineai_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<engineai_ros2::EngineAICmdVelControllerNode>();
  engineai_ros2::g_node_weak = node;

  std::signal(SIGINT, engineai_ros2::signal_handler);
  std::signal(SIGTERM, engineai_ros2::signal_handler);
  std::signal(SIGSEGV, engineai_ros2::signal_handler);

  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    node->publish_stop();
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Unknown fatal error");
    node->publish_stop();
  }

  rclcpp::shutdown();
  return 0;
}
