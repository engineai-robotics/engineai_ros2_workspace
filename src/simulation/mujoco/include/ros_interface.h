#ifndef MUJOCO_ROS_INTERFACE_H_
#define MUJOCO_ROS_INTERFACE_H_

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/motion_state.hpp"
#include "rclcpp/rclcpp.hpp"

// MuJoCo includes
#include <mujoco/mujoco.h>

// Forward declarations
namespace mujoco {
class Simulate;
}
class ConfigLoader;

namespace mujoco {

class RosInterface {
 public:
  RosInterface(const rclcpp::Node::SharedPtr& node, std::shared_ptr<ConfigLoader> config_loader);
  ~RosInterface();

  // Initialize the MuJoCo interface
  bool Initialize();

  // Callback for joint command messages
  void JointCommandCallback(const interface_protocol::msg::JointCommand::SharedPtr msg);

  // Update the simulation state to publish to ROS
  void UpdateSimState(const mjModel* m, mjData* d);

  // Get joint command values (thread-safe)
  interface_protocol::msg::JointCommand GetCommandedSafe();

  // Set the current mjModel and mjData
  void SetModelAndData(mjModel* model, mjData* data);

  // Get the ROS node
  rclcpp::Node::SharedPtr GetNode() const { return node_; }

 private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<interface_protocol::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<interface_protocol::msg::ImuInfo>::SharedPtr imu_pub_;
  rclcpp::Publisher<interface_protocol::msg::MotionState>::SharedPtr motion_state_pub_;

  // Subscribers
  rclcpp::Subscription<interface_protocol::msg::JointCommand>::SharedPtr joint_cmd_sub_;

  // Config loader
  std::shared_ptr<ConfigLoader> config_loader_;

  // Number of joints
  int num_total_joints_ = 0;

  // Current joint command
  interface_protocol::msg::JointCommand joint_command_;

  // MuJoCo model and data
  mjModel* model_;
  mjData* data_;

  // Timer for publishing motion state
  rclcpp::TimerBase::SharedPtr motion_state_timer_;
  
  // Motion state timer callback
  void MotionStateTimerCallback();

  // Mutex for thread safety
  std::mutex mtx_;

  // Flag indicating if we have a floating base robot
  bool is_floating_base_;
};

}  // namespace mujoco

#endif  // MUJOCO_ROS_INTERFACE_H_
