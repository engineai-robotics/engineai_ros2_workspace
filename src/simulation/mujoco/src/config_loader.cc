#include "config_loader.h"
#include <fstream>
#include <iostream>

ConfigLoader::ConfigLoader(const std::string& config_file) : config_file_(config_file) {}

bool ConfigLoader::LoadConfig() {
  try {
    // Load YAML file
    YAML::Node config = YAML::LoadFile(config_file_);

    // Load model parameters
    urdf_filename_ = config["urdf"].as<std::string>();
    xml_filename_ = config["xml"].as<std::string>();

    if (config["model_param"]) {
      num_total_joints_ = config["model_param"]["num_total_joints"].as<int>();
      num_contacts_ = config["model_param"]["num_contacts"].as<int>();
      num_single_contact_dimensions_ = config["model_param"]["num_single_contact_dimensions"].as<int>();
    }

    // Load topic names
    if (config["sensor"] && config["sensor"]["imu_topic"]) {
      imu_topic_ = config["sensor"]["imu_topic"].as<std::string>();
    }

    if (config["actuator"]) {
      if (config["actuator"]["joint_state_topic"]) {
        joint_state_topic_ = config["actuator"]["joint_state_topic"].as<std::string>();
      }

      if (config["actuator"]["joint_command_topic"]) {
        joint_command_topic_ = config["actuator"]["joint_command_topic"].as<std::string>();
      }
    }

    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error loading config file: " << e.what() << std::endl;
    return false;
  }
}

std::string ConfigLoader::GetModelFilePath() const { return assets_path_ + "/resource/" + xml_filename_; }

std::string ConfigLoader::GetResourceDir() const { return assets_path_ + "/resource"; }
