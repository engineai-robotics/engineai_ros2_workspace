#include <rclcpp/rclcpp.hpp>
#include "sim_manager.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto& sim_manager = SimManager::GetInstance();

  if (!sim_manager.Initialize()) {
    return 1;
  }

  sim_manager.Run();
  rclcpp::shutdown();
  return 0;
}
