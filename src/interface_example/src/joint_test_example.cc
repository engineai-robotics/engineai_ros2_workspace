#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "components/message_handler.hpp"

using namespace std::chrono_literals;

namespace {
// Constants Default
constexpr double kControlFrequency = 500.0;                 // Control frequency in Hz
constexpr double kControlPeriod = 1.0 / kControlFrequency;  // Control period in seconds
}  // namespace

class JointTestExample : public rclcpp::Node {
 public:
  JointTestExample(const std::string& config_dir_path)
      : Node("joint_test_example"), joint_test_config_path_(config_dir_path + "/joint_test.yaml") {
    // Load motor parameters from yaml
    LoadJointTestParams();
  }

  bool Initialize() {
    try {
      // Initialize message handler
      msg_handler_ = std::make_shared<example::MessageHandler>(shared_from_this());
      msg_handler_->Initialize();

      // Wait for first motor state
      while (!msg_handler_->GetLatestJointState()) {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for first motor state...");
      }

      // Get initial motor state
      auto initial_state = msg_handler_->GetLatestJointState();
      initial_positions_ = initial_state->position;

      // Print initial positions
      std::stringstream ss;
      ss << "\nInitial positions:\n[";
      for (size_t i = 0; i < initial_positions_.size(); ++i) {
        ss << std::fixed << std::setprecision(3) << initial_positions_[i];
        if (i < initial_positions_.size() - 1) {
          ss << ", ";
          // Add newline every 6 elements for better readability
          if ((i + 1) % 6 == 0) {
            ss << "\n ";
          }
        }
      }
      ss << "]\n";
      RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

      // Convert initial motor positions to joint positions and generate trajectories
      GenerateTrajectories();

      // Create timer for periodic execution
      timer_ = create_wall_timer(std::chrono::duration<double>(kControlPeriod),
                                 std::bind(&JointTestExample::TimerCallback, this));

      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize: %s", e.what());
      return false;
    }
  }

 private:
  void GenerateTrajectories() {
    // Convert initial motor positions to joint positions
    current_joint_positions_ = Eigen::VectorXd(initial_positions_.size());
    for (size_t i = 0; i < initial_positions_.size(); ++i) {
      current_joint_positions_[i] = initial_positions_[i];
    }

    // Initialize interpolation for each joint
    interpolated_positions_.resize(initial_positions_.size());
    current_steps_.resize(initial_positions_.size(), 0);
    reached_targets_.resize(initial_positions_.size(), false);

    // Generate interpolated trajectories for each joint
    for (size_t i = 0; i < initial_positions_.size(); ++i) {
      interpolated_positions_[i].resize(num_steps_[i]);
      double step = (target_positions_[i] - current_joint_positions_[i]) / (num_steps_[i] - 1);
      for (int j = 0; j < num_steps_[i]; ++j) {
        interpolated_positions_[i][j] = current_joint_positions_[i] + step * j;
      }
    }

    RCLCPP_INFO(get_logger(), "Trajectories generated for %ld joints", initial_positions_.size());
  }

  void LoadJointTestParams() {
    try {
      YAML::Node config = YAML::LoadFile(joint_test_config_path_);

      // Load parameters
      auto target_pos = config["target_position"];
      auto steps = config["num_steps"];
      auto kp = config["kp"];
      auto kd = config["kd"];

      // Clear vectors
      target_positions_.clear();
      num_steps_.clear();
      kp_.clear();
      kd_.clear();

      // Concatenate all groups into single vectors
      for (const auto& pos_group : target_pos) {
        auto vec = pos_group.as<std::vector<double>>();
        target_positions_.insert(target_positions_.end(), vec.begin(), vec.end());
      }

      for (const auto& steps_group : steps) {
        auto vec = steps_group.as<std::vector<double>>();
        num_steps_.insert(num_steps_.end(), vec.begin(), vec.end());
      }

      for (const auto& kp_group : kp) {
        auto vec = kp_group.as<std::vector<double>>();
        kp_.insert(kp_.end(), vec.begin(), vec.end());
      }

      for (const auto& kd_group : kd) {
        auto vec = kd_group.as<std::vector<double>>();
        kd_.insert(kd_.end(), vec.begin(), vec.end());
      }

      // Print loaded parameters for verification
      RCLCPP_INFO(get_logger(), "Loaded joint test parameters:");
      RCLCPP_INFO(get_logger(), "Number of joints: %ld", target_positions_.size());

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load joint test parameters: %s", e.what());
      throw;
    }
  }

  void TimerCallback() {
    auto joint_command = std::make_shared<interface_protocol::msg::JointCommand>();
    joint_command->position.resize(initial_positions_.size());
    joint_command->velocity.resize(initial_positions_.size());
    joint_command->feed_forward_torque.resize(initial_positions_.size(), 0.0);
    joint_command->torque.resize(initial_positions_.size(), 0.0);
    joint_command->stiffness.resize(initial_positions_.size(), 400.0);
    joint_command->damping.resize(initial_positions_.size(), 5.0);

    bool all_reached = true;
    for (size_t i = 0; i < initial_positions_.size(); ++i) {
      if (!reached_targets_[i]) {
        if (current_steps_[i] < num_steps_[i]) {
          double joint_pos = interpolated_positions_[i][current_steps_[i]];
          joint_command->position[i] = joint_pos;
          joint_command->velocity[i] = 0.0;  // Zero velocity for position control
          current_steps_[i]++;
        } else {
          // Hold at target position
          double joint_pos = target_positions_[i];
          joint_command->position[i] = joint_pos;
          joint_command->velocity[i] = 0.0;
          reached_targets_[i] = true;
        }
        all_reached = false;
      } else {
        // Keep sending the target position
        double joint_pos = target_positions_[i];
        joint_command->position[i] = joint_pos;
        joint_command->velocity[i] = 0.0;
      }
    }
    // Print initial positions
    std::stringstream ss;
    ss << "\nCurrent positions:\n[";
    for (size_t i = 0; i < joint_command->position.size(); ++i) {
      ss << std::fixed << std::setprecision(3) << joint_command->position[i];
      if (i < joint_command->position.size() - 1) {
        ss << ", ";
        // Add newline every 6 elements for better readability
        if ((i + 1) % 6 == 0) {
          ss << "\n ";
        }
      }
    }
    ss << "]\n";
    // RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    if (all_reached) {
      RCLCPP_INFO_ONCE(get_logger(), "All joints have reached their target positions!");
      return;
    }

    msg_handler_->PublishJointCommand(joint_command);
  }

  std::string config_path_;
  std::string joint_test_config_path_;
  std::shared_ptr<example::MessageHandler> msg_handler_;
  std::vector<double> initial_positions_;

  // Joint test parameters
  std::vector<double> target_positions_;
  std::vector<double> num_steps_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<std::vector<double>> interpolated_positions_;
  std::vector<int> current_steps_;
  std::vector<bool> reached_targets_;
  Eigen::VectorXd current_joint_positions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_config_directory>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  // Remove trailing slash if present
  std::string config_dir = argv[1];
  if (!config_dir.empty() && config_dir.back() == '/') {
    config_dir.pop_back();
  }

  auto node = std::make_shared<JointTestExample>(config_dir);
  if (!node->Initialize()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}