#ifndef CONFIG_LOADER_H_
#define CONFIG_LOADER_H_

#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <vector>

class ConfigLoader {
 public:
  ConfigLoader(const std::string& config_file);
  ~ConfigLoader() = default;

  bool LoadConfig();

  // Getters for various configuration parameters
  std::string GetUrdfFilename() const { return urdf_filename_; }
  std::string GetXmlFilename() const { return xml_filename_; }
  int GetNumTotalJoints() const { return num_total_joints_; }
  int GetNumContacts() const { return num_contacts_; }
  int GetNumSingleContactDimensions() const { return num_single_contact_dimensions_; }

  // Topic names
  std::string GetImuTopic() const { return imu_topic_; }
  std::string GetJointStateTopic() const { return joint_state_topic_; }
  std::string GetJointCommandTopic() const { return joint_command_topic_; }

  // Asset path related methods
  std::string GetModelFilePath() const;
  std::string GetResourceDir() const;
  void SetAssetsPath(const std::string& assets_path) { assets_path_ = assets_path; }

 private:
  std::string config_file_;
  std::string assets_path_;  // Path to assets directory

  // Model parameters
  std::string urdf_filename_;
  std::string xml_filename_;
  int num_total_joints_ = 0;
  int num_contacts_ = 0;
  int num_single_contact_dimensions_ = 0;

  // Topic names
  std::string imu_topic_;
  std::string joint_state_topic_;
  std::string joint_command_topic_;
};

#endif  // CONFIG_LOADER_H_
