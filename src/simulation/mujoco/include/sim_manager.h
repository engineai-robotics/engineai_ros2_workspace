#pragma once

#include <mujoco/mujoco.h>
#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string_view>
#include <thread>
#include "config_loader.h"
#include "ros_interface.h"
#include "simulate/simulate.h"

class SimManager {
 public:
  // Get the singleton instance
  static SimManager& GetInstance();

  // Delete copy constructor and assignment operator
  SimManager(const SimManager&) = delete;
  SimManager& operator=(const SimManager&) = delete;

  // Initialize the simulation
  bool Initialize();

  // Run the simulation
  void Run();

  // Controller callback used by MuJoCo
  void TorqueController(const mjModel* m, mjData* d);

 private:
  // Private constructor for singleton
  SimManager();
  ~SimManager();

  // Private member functions
  void PhysicsThread(std::string_view filename);
  void PhysicsLoop();
  mjModel* LoadModel(std::string_view file);
  const char* Diverged(int disableflags, const mjData* d);
  void HandleDropLoad();
  void HandleUILoad();

  // Private member variables
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ConfigLoader> config_loader_;
  std::unique_ptr<mujoco::RosInterface> ros_interface_;
  std::unique_ptr<mujoco::Simulate> sim_;
  std::thread physics_thread_;

  // MuJoCo model and data
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  std::array<char, 1024> mj_load_error_;
};