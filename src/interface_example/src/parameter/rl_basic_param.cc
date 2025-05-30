#include "parameter/rl_basic_param.h"

#include <iostream>

namespace example {

RlBasicParam::RlBasicParam(const std::string& config_file) {
  LoadFromYaml(config_file);

  // Initialize derived parameters
  num_actions = active_joint_names.size();

  // Initialize observation scale vector with correct size
  observation_scale = Eigen::VectorXd::Zero(num_observations);

  observation_scale <<
      Eigen::VectorXd::Constant(num_actions, observation_scale_dof_pos),  // joint position - joint default position
      Eigen::VectorXd::Constant(num_actions, observation_scale_dof_vel),  // joint velocity
      Eigen::VectorXd::Ones(num_actions),                                 // last joint action
      Eigen::Vector3d::Constant(observation_scale_angular_vel),           // base angular velocity
      Eigen::Vector3d::Constant(observation_scale_quat);                  // base euler angle xyz
  obs_commands_scale = Eigen::VectorXd::Zero(num_commands);
  obs_commands_scale << Eigen::Vector2d::Constant(observation_scale_linear_vel),  // linear velocity command
      observation_scale_angular_vel;                                          // angular velocity command
  
}

void RlBasicParam::LoadFromYaml(const std::string& config_file) {
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    // mix = config["mix"].as<bool>();
    // Load MLP net parameters
    policy_file = config["policy_file"].as<std::string>();
    num_observations = config["num_observations"].as<int>();
    active_joint_names = config["active_joint_names"].as<std::vector<std::string>>();
    active_joint_idx = LoadIntVectorFromYaml(config["active_joint_idx"]);
    num_include_obs_steps = config["num_include_obs_steps"].as<int>();

    // Load observation parameters
    observation_scale_linear_vel = config["observation_scale_linear_vel"].as<double>();
    observation_scale_angular_vel = config["observation_scale_angular_vel"].as<double>();
    observation_scale_dof_pos = config["observation_scale_dof_pos"].as<double>();
    observation_scale_dof_vel = config["observation_scale_dof_vel"].as<double>();
    observation_scale_quat = config["observation_scale_quat"].as<double>();
    observation_clip = config["observation_clip"].as<double>();
    
    // Load remote command parameters
    remote_command_sampling_frequency = config["remote_command_sampling_frequency"].as<double>();
    remote_command_cut_off_frequency = config["remote_command_cut_off_frequency"].as<double>();

    // Load gait parameters
    cycle_time = config["cycle_time"].as<double>();
    transition_time = config["transition_time"].as<double>();
    // Load joint control parameters
    action_clip = config["action_clip"].as<double>();
    default_joint_q = LoadVectorArrayFromYaml(config["default_joint_q"]);
    joint_kp = LoadVectorArrayFromYaml(config["joint_kp"]);
    joint_kd = LoadVectorArrayFromYaml(config["joint_kd"]);
    action_scale = LoadVectorArrayFromYaml(config["action_scale"]);
    control_dt = config["control_dt"].as<double>();
    imu_install_delta_bias = config["imu_install_delta_bias"].as<double>();
    imu_install_bias = LoadVectorFromYaml(config["imu_install_bias"]);
    // Load command scale
    auto command_scale_node = config["command_scale"];
    command_scale = Eigen::Vector3d(command_scale_node[0].as<double>(), command_scale_node[1].as<double>(),
                                    command_scale_node[2].as<double>());
    num_commands = config["num_commands"].as<int>();
    num_clock_signal = config["num_clock_signal"].as<int>();

  } catch (const YAML::Exception& e) {
    std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    throw;
  }
  std::cout << "LoadFromYaml done" << std::endl;
}

Eigen::VectorXd RlBasicParam::LoadVectorFromYaml(const YAML::Node& node) {
  std::vector<double> vec;
  for (const auto& item : node) {
    vec.push_back(item.as<double>());
  }
  return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
}

Eigen::VectorXi RlBasicParam::LoadIntVectorFromYaml(const YAML::Node& node) {
  std::vector<int> vec;
  for (const auto& item : node) {
    vec.push_back(item.as<int>());
  }
  return Eigen::Map<Eigen::VectorXi>(vec.data(), vec.size());
}
std::vector<Eigen::VectorXd> RlBasicParam::LoadVectorArrayFromYaml(const YAML::Node& node) {
  std::vector<Eigen::VectorXd> result;
  for (const auto& item : node) {
    result.push_back(LoadVectorFromYaml(item));
  }
  return result;
}

}  // namespace example