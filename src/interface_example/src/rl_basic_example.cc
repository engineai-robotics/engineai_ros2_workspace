#include <chrono>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "components/message_handler.hpp"
#include "math/concatenate_vector.h"
#include "math/mnn_model.h"
#include "math/rotation_matrix.h"
#include "parameter/rl_basic_param.h"

using namespace std::chrono_literals;

namespace example {

class RlBasicRunner : public rclcpp::Node {
 public:
  explicit RlBasicRunner(const std::string& config_file_dir) : Node("rl_basic_runner") {
    std::string config_file = config_file_dir + "/rl_basic_param.yaml";
    param_ = std::make_shared<RlBasicParam>(config_file);
    config_file_dir_ = config_file_dir;
    joint_command_ = std::make_shared<interface_protocol::msg::JointCommand>();
  }

  bool Initialize() {
    try {
      // Initialize message handler
      message_handler_ = std::make_shared<MessageHandler>(shared_from_this());
      message_handler_->Initialize();
      // Wait for first motion state
      while (!message_handler_->GetLatestMotionState() ||
             message_handler_->GetLatestMotionState()->current_motion_task != "joint_bridge") {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for joint bridge state...");
      }
      RCLCPP_INFO(get_logger(), "Already in joint bridge state");
      // Get initial joint positions
      auto initial_state = message_handler_->GetLatestJointState();
      if (!initial_state) {
        RCLCPP_ERROR(get_logger(), "Failed to get initial joint state");
        return false;
      }

      initial_joint_q_ =
          Eigen::Map<const Eigen::VectorXd>(initial_state->position.data(), initial_state->position.size());

      RCLCPP_INFO_STREAM(get_logger(), "Initial joint positions: " << initial_joint_q_.transpose());

      // Initialize active joint indices
      active_joint_idx_ = param_->active_joint_idx;
      // Concatenate joint parameters from yaml
      default_joint_q_ = math::ConcatenateVectors(param_->default_joint_q);
      joint_kp_ = math::ConcatenateVectors(param_->joint_kp);
      joint_kd_ = math::ConcatenateVectors(param_->joint_kd);
      action_scale_ = math::ConcatenateVectors(param_->action_scale);

      // Initialize MNN model
      mlp_net_ = std::make_unique<math::MnnModel>(config_file_dir_ + "/" + param_->policy_file);
      mlp_net_observation_.setZero(param_->num_observations, param_->num_include_obs_steps);
      mlp_net_action_.setZero(param_->num_actions);

      // Initialize control variables
      time_ = 0.0;
      global_phase_ = 0.0;
      is_first_time_ = true;

      RCLCPP_INFO(get_logger(), "Starting control loop");
      // Create control timer
      control_timer_ = create_wall_timer(std::chrono::duration<double>(param_->control_dt),
                                         std::bind(&RlBasicRunner::ControlCallback, this));

      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize: %s", e.what());
      return false;
    }
  }

 private:
  void ControlCallback() {
    if (message_handler_->GetLatestMotionState()->current_motion_task != "joint_bridge") {
      time_ = 0.0;
      is_first_time_ = true;
      return;
    }
    auto joint_state = message_handler_->GetLatestJointState();
    if (!joint_state) return;  // Skip if no joint state received yet

    UpdateState(joint_state);
    CalculateObservation();
    CalculateMotorCommand();
    SendMotorCommand();

    time_ += param_->control_dt;
  }

  void UpdateState(const interface_protocol::msg::JointState::SharedPtr& joint_state) {
    // Update joint states
    q_real_ = Eigen::Map<const Eigen::VectorXd>(joint_state->position.data(), joint_state->position.size());
    qd_real_ = Eigen::Map<const Eigen::VectorXd>(joint_state->velocity.data(), joint_state->velocity.size());

    // Update command from gamepad
    auto gamepad = message_handler_->GetLatestGamepad();
    if (gamepad) {
      command_.x() =
          gamepad->analog_states[interface_protocol::msg::GamepadKeys::LEFT_STICK_X] * param_->command_scale.x();
      command_.y() =
          gamepad->analog_states[interface_protocol::msg::GamepadKeys::LEFT_STICK_Y] * param_->command_scale.y();
      command_.z() =
          gamepad->analog_states[interface_protocol::msg::GamepadKeys::RIGHT_STICK_Y] * param_->command_scale.z();
    }
  }

  void CalculateObservation() {
    // Calculate phase info
    global_phase_ += param_->control_dt / param_->cycle_time;
    global_phase_ -= static_cast<int>(global_phase_);



    // Get IMU data
    auto imu = message_handler_->GetLatestImu();
    Eigen::Matrix3d R_real =
        Eigen::Quaterniond(imu->quaternion.w, imu->quaternion.x, imu->quaternion.y, imu->quaternion.z)
            .toRotationMatrix();
    Eigen::Vector3d w_real = Eigen::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    // ZYX顺序的欧拉角（等同于RPY）
    Eigen::Vector3d euler_xyz = math::CalcRollPitchYawFromRotationMatrix(R_real);

    // Stack the observation
    Eigen::VectorXd mlp_net_observation_single = Eigen::VectorXd::Zero(param_->num_observations);
    // if (param_->mix) {
    //   mlp_net_observation_single <<                         //  command
    //       (q_real_ - default_joint_q_)(active_joint_idx_),  //  joint position - joint default position: kDoFs
    //       qd_real_(active_joint_idx_),                      //  joint velocity: kDoFs
    //       mlp_net_action_,                                  //  last joint action: kDoFs
    //       w_real,                                           //  base angular velocity w.r.t base frame: 3
    //       euler_xyz;                                        //  base euler angle rpy w.r.t base frame: 3
    // } else {
    //   mlp_net_observation_single << clock_signal,           //  phase signals: 2
    //       command_,                                         //  command
    //       (q_real_ - default_joint_q_)(active_joint_idx_),  //  joint position - joint default position: kDoFs
    //       qd_real_(active_joint_idx_),                      //  joint velocity: kDoFs
    //       mlp_net_action_,                                  //  last joint action: kDoFs
    //       w_real,                                           //  base angular velocity w.r.t base frame: 3
    //       euler_xyz;                                        //  base euler angle rpy w.r.t base frame: 3
    // }
    mlp_net_observation_single << (q_real_ - default_joint_q_)(
      active_joint_idx_),           //  joint position - joint default position: kDoFs
      qd_real_(active_joint_idx_),  //  joint velocity: kDoFs
      mlp_net_action_,              //  last joint action: kDoFs
      w_real,                       //  base angular velocity w.r.t base frame: 3
      euler_xyz;                    //  base euler angle rpy w.r.t base frame: 3

  // Scales and clips the observation

    // Scale and clip the observation
    mlp_net_observation_single.array() *= param_->observation_scale.array();
    mlp_net_observation_single =
        mlp_net_observation_single.cwiseMax(-param_->observation_clip).cwiseMin(param_->observation_clip);

    // Update the observation buffer
    if (is_first_time_) {
      is_first_time_ = false;
      mlp_net_observation_.setZero(param_->num_observations, param_->num_include_obs_steps);
      mlp_net_action_.setZero(param_->num_actions);
      mlp_net_observation_.rightCols(1) = mlp_net_observation_single;
    } else {
      mlp_net_observation_.leftCols(param_->num_include_obs_steps - 1) =
          mlp_net_observation_.rightCols(param_->num_include_obs_steps - 1);
      mlp_net_observation_.rightCols(1) = mlp_net_observation_single;
    }
  }

  void CalculateMotorCommand() {
    Eigen::Vector2d clock_signal(std::sin(2 * M_PI * global_phase_), std::cos(2 * M_PI * global_phase_));
    Eigen::VectorXd obs = Eigen::VectorXd::Zero(
      param_->num_observations * param_->num_include_obs_steps + 
      param_->num_clock_signal + 
      param_->num_commands
    );
    obs = Eigen::VectorXd::Zero(param_->num_observations * param_->num_include_obs_steps + param_->num_clock_signal + param_->num_commands);
    obs.head(param_->num_observations * param_->num_include_obs_steps) =
        Eigen::Map<Eigen::VectorXd>(mlp_net_observation_.transpose().data(), mlp_net_observation_.size());
    command_.array() *= param_->obs_commands_scale.array();
    obs.tail(param_->num_clock_signal + param_->num_commands) << clock_signal, command_;


    // Get MNN output
    mlp_net_action_ = mlp_net_->Inference(obs.cast<float>()).cast<double>();
    mlp_net_action_ = mlp_net_action_.cwiseMax(-param_->action_clip).cwiseMin(param_->action_clip);

    // Calculate desired joint positions
    q_des_ = default_joint_q_;                                                 // Use concatenated default_joint_q_
    q_des_(active_joint_idx_) += mlp_net_action_.cwiseProduct(action_scale_);  // Use concatenated action_scale_

    if (time_ < param_->transition_time) {
      float ratio = time_ / param_->transition_time;
      q_des_ = ratio * q_des_ + (1 - ratio) * initial_joint_q_;
    }
  }

  void SendMotorCommand() {
    // Convert Eigen vectors to std::vector
    joint_command_->position = std::vector<double>(q_des_.data(), q_des_.data() + q_des_.size());
    joint_command_->velocity = std::vector<double>(q_des_.size(), 0.0);
    joint_command_->feed_forward_torque = std::vector<double>(q_des_.size(), 0.0);
    joint_command_->torque = std::vector<double>(q_des_.size(), 0.0);
    joint_command_->stiffness = std::vector<double>(joint_kp_.data(), joint_kp_.data() + joint_kp_.size());
    joint_command_->damping = std::vector<double>(joint_kd_.data(), joint_kd_.data() + joint_kd_.size());
    joint_command_->parallel_parser_type = interface_protocol::msg::ParallelParserType::RL_PARSER;
    // Send command through message handler
    message_handler_->PublishJointCommand(*joint_command_);
  }

  // Parameters
  std::shared_ptr<RlBasicParam> param_;

  // Message handling
  std::shared_ptr<MessageHandler> message_handler_;

  // MNN model
  std::unique_ptr<math::MnnModel> mlp_net_;
  Eigen::MatrixXd mlp_net_observation_;
  Eigen::VectorXd mlp_net_action_;

  // State variables
  double time_;
  double global_phase_;
  bool is_first_time_;
  Eigen::Vector3d command_;
  Eigen::VectorXd q_real_;
  Eigen::VectorXd qd_real_;
  Eigen::VectorXd q_des_;
  Eigen::VectorXi active_joint_idx_;
  Eigen::VectorXd initial_joint_q_;
  Eigen::VectorXd default_joint_q_;
  Eigen::VectorXd joint_kp_;
  Eigen::VectorXd joint_kd_;
  Eigen::VectorXd action_scale_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::string config_file_dir_;
  interface_protocol::msg::JointCommand::SharedPtr joint_command_;
};

}  // namespace example

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rl_basic_example"), "Usage: rl_basic_example <config_file_dir>");
    return 1;
  }

  auto node = std::make_shared<example::RlBasicRunner>(argv[1]);
  if (!node->Initialize()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
