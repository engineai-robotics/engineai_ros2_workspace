#include "message_handler.hpp"
#include <rclcpp/qos.hpp>

namespace example {

MessageHandler::MessageHandler(rclcpp::Node::SharedPtr node) : node_(node) {}

void MessageHandler::Initialize() {
  rclcpp::QoS qos(3);
  qos.best_effort();
  qos.durability_volatile();
  // Initialize subscribers
  gamepad_sub_ = node_->create_subscription<interface_protocol::msg::GamepadKeys>(
      "/hardware/gamepad_keys", qos, std::bind(&MessageHandler::GamepadCallback, this, std::placeholders::_1));

  imu_sub_ = node_->create_subscription<interface_protocol::msg::ImuInfo>(
      "/hardware/imu_info", qos, std::bind(&MessageHandler::ImuCallback, this, std::placeholders::_1));

  joint_state_sub_ = node_->create_subscription<interface_protocol::msg::JointState>(
      "/hardware/joint_state", qos, std::bind(&MessageHandler::JointStateCallback, this, std::placeholders::_1));

  // Initialize publisher
  joint_cmd_pub_ = node_->create_publisher<interface_protocol::msg::JointCommand>("/hardware/joint_command", qos);

  motion_state_sub_ = node_->create_subscription<interface_protocol::msg::MotionState>(
      "/motion/motion_state", 1, std::bind(&MessageHandler::MotionStateCallback, this, std::placeholders::_1));
}



void MessageHandler::MotionStateCallback(const interface_protocol::msg::MotionState::SharedPtr msg) {
  latest_motion_state_ = msg;
}

void MessageHandler::GamepadCallback(const interface_protocol::msg::GamepadKeys::SharedPtr msg) {
  latest_gamepad_ = msg;
}

void MessageHandler::ImuCallback(const interface_protocol::msg::ImuInfo::SharedPtr msg) { latest_imu_ = msg; }

void MessageHandler::JointStateCallback(const interface_protocol::msg::JointState::SharedPtr msg) {
  latest_joint_state_ = msg;
}

void MessageHandler::PublishJointCommand(const interface_protocol::msg::JointCommand::SharedPtr command) {
  joint_cmd_pub_->publish(*command);
}
void MessageHandler::PublishJointCommand(const interface_protocol::msg::JointCommand& command) {
  joint_cmd_pub_->publish(command);
}
}  // namespace example
