#ifndef EXAMPLE_MESSAGE_HANDLER_HPP_
#define EXAMPLE_MESSAGE_HANDLER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "interface_protocol/msg/gamepad_keys.hpp"
#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/motion_state.hpp"
#include "interface_protocol/msg/parallel_parser_type.hpp"
namespace example {

class MessageHandler {
 public:
  explicit MessageHandler(rclcpp::Node::SharedPtr node);
  ~MessageHandler() = default;

  void Initialize();

  void PublishJointCommand(const interface_protocol::msg::JointCommand::SharedPtr command);
  void PublishJointCommand(const interface_protocol::msg::JointCommand& command);

  // Get latest states
  interface_protocol::msg::JointState::SharedPtr GetLatestJointState() const { return latest_joint_state_; }
  interface_protocol::msg::GamepadKeys::SharedPtr GetLatestGamepad() const { return latest_gamepad_; }
  interface_protocol::msg::ImuInfo::SharedPtr GetLatestImu() const { return latest_imu_; }
  interface_protocol::msg::MotionState::SharedPtr GetLatestMotionState() const { return latest_motion_state_; }

 private:
  void GamepadCallback(const interface_protocol::msg::GamepadKeys::SharedPtr msg);
  void ImuCallback(const interface_protocol::msg::ImuInfo::SharedPtr msg);
  void JointStateCallback(const interface_protocol::msg::JointState::SharedPtr msg);
  void MotionStateCallback(const interface_protocol::msg::MotionState::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;

  // Subscribers
  rclcpp::Subscription<interface_protocol::msg::GamepadKeys>::SharedPtr gamepad_sub_;
  rclcpp::Subscription<interface_protocol::msg::ImuInfo>::SharedPtr imu_sub_;
  rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<interface_protocol::msg::MotionState>::SharedPtr motion_state_sub_;
  // Publisher
  rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_pub_;

  // Message storage
  interface_protocol::msg::GamepadKeys::SharedPtr latest_gamepad_;
  interface_protocol::msg::ImuInfo::SharedPtr latest_imu_;
  interface_protocol::msg::JointState::SharedPtr latest_joint_state_;
  interface_protocol::msg::MotionState::SharedPtr latest_motion_state_;
};

}  // namespace example

#endif  // EXAMPLE_MESSAGE_HANDLER_HPP_
