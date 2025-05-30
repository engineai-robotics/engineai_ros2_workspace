cmake_minimum_required(VERSION 3.16)

project(mujoco_simulator LANGUAGES C CXX)

# Set variable paths similar to the LCM version
set(SIMULATION_MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(ENGINEAI_ROBOTICS_THIRD_PARTY /opt/engineai_robotics_third_party)

# Set cmake path
list(APPEND CMAKE_MODULE_PATH ${SIMULATION_MUJOCO_DIR}/simulate/cmake)
list(APPEND CMAKE_PREFIX_PATH ${ENGINEAI_ROBOTICS_THIRD_PARTY})

# Set ROS2 build type
set(CMAKE_BUILD_TYPE Release CACHE STRING "Set build type as Release." FORCE)

# Some necessary settings, match LCM version
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set cmake paths
list(APPEND CMAKE_MODULE_PATH ${SIMULATION_MUJOCO_DIR}/simulate/cmake)
list(APPEND CMAKE_PREFIX_PATH ${ENGINEAI_ROBOTICS_THIRD_PARTY})

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface_protocol REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find third-party packages exactly like the LCM version
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

# Set explicit library paths for key dependencies
set(MUJOCO_LIB "${ENGINEAI_ROBOTICS_THIRD_PARTY}/lib/libmujoco.so")

# Configure the MuJoCo simulation library
set(SIMULATE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/simulate)

# Add directory with proper configuration
add_subdirectory(${SIMULATE_DIR})

# Include directories for the main project
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}  # Add parent directory to correctly resolve simulate/array_safety.h
  ${SIMULATE_DIR}
  ${ENGINEAI_ROBOTICS_THIRD_PARTY}/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Full path to the third-party libraries
link_directories(${ENGINEAI_ROBOTICS_THIRD_PARTY}/lib)



# ROS2 executable
add_executable(mujoco_simulator
  src/main.cc
  src/ros_interface.cc
  src/config_loader.cc
  src/sim_manager.cc
)

target_include_directories(mujoco_simulator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}  # Add parent directory to correctly resolve simulate/array_safety.h
  ${SIMULATE_DIR}
  ${ENGINEAI_ROBOTICS_THIRD_PARTY}/include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(mujoco_simulator
  ${MUJOCO_LIB}
  libsimulate
  yaml-cpp::yaml-cpp
  ${EIGEN3_LIBRARIES}
  glfw
  pthread
  ${CMAKE_DL_LIBS}
  ${rclcpp_LIBRARIES}
  ${rcl_LIBRARIES}
  ${rcutils_LIBRARIES}
)

ament_target_dependencies(mujoco_simulator
  rclcpp
  interface_protocol
  std_msgs
  geometry_msgs
)

install(TARGETS
  mujoco_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  assets
  DESTINATION share/${PROJECT_NAME}
)

ament_package()