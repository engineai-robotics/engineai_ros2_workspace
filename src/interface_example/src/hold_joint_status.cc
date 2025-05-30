#include <chrono>
#include <interface_protocol/msg/detail/motor_command__struct.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "components/message_handler.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("joint_test_example");
  auto message_handler = std::make_shared<example::MessageHandler>(node);
  message_handler->Initialize();

  interface_protocol::msg::JointCommand hold_joint_command;

  while (!message_handler->GetLatestJointState()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Waiting for first joint state...");
  }
  auto latest_joint_state = message_handler->GetLatestJointState();
  //print latest_joint_state
  std::stringstream ss;
  ss << "Latest joint state size: " << latest_joint_state->position.size() << "\nvalues: " ;
  for (size_t i = 0; i < latest_joint_state->position.size(); i++) {
    ss << " " << latest_joint_state->position[i];
  }
  RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());  

  // Example: Copy current positions and add some feed-forward torque
  // Example: Copy current positions and add some feed-forward torque
  hold_joint_command.position = latest_joint_state->position;
  hold_joint_command.feed_forward_torque = latest_joint_state->torque;
  hold_joint_command.torque = latest_joint_state->torque;
  hold_joint_command.velocity.resize(latest_joint_state->position.size(), 0.0);
  hold_joint_command.stiffness.resize(latest_joint_state->position.size(), 400.0);  // Example stiffness
  hold_joint_command.damping.resize(latest_joint_state->position.size(), 5.0);      // Example damping

  // Create timer for publishing motor commands at 500Hz
  rclcpp::TimerBase::SharedPtr timer =
      node->create_wall_timer(2ms,  // 500Hz = 2ms period
                              [&]() {
                                hold_joint_command.header.stamp = node->get_clock()->now();
                                message_handler->PublishJointCommand(hold_joint_command);
                              });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
