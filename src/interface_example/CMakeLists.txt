cmake_minimum_required(VERSION 3.8)
project(interface_example)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND ENGINEAI_ROBOTICS_THIRD_PARTY /opt/engineai_robotics_third_party)
list(APPEND CMAKE_PREFIX_PATH ${ENGINEAI_ROBOTICS_THIRD_PARTY})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interface_protocol REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include 
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Add executable with message_handler source
add_executable(hold_joint_status
  src/hold_joint_status.cc
  src/components/message_handler.cc
)
ament_target_dependencies(hold_joint_status
  rclcpp
  interface_protocol
)

# Add joint_test_example executable
add_executable(joint_test_example
  src/joint_test_example.cc
  src/components/message_handler.cc
)
ament_target_dependencies(joint_test_example
  rclcpp
  interface_protocol
)
target_link_libraries(joint_test_example
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
)

# Search source file in src/parameter/ and src/components/ and src/math/
file(GLOB_RECURSE rl_basic_example_src_files
  src/parameter/*.cc
  src/components/*.cc
  src/math/*.cc
  src/math/*.cpp
)

add_executable(rl_basic_example
  src/rl_basic_example.cc
  ${rl_basic_example_src_files}
)

target_include_directories(rl_basic_example
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    ${ENGINEAI_ROBOTICS_THIRD_PARTY}/include
)

target_link_libraries(rl_basic_example
  yaml-cpp::yaml-cpp
  Eigen3::Eigen
  ${ENGINEAI_ROBOTICS_THIRD_PARTY}/lib/libMNN.so
)

ament_target_dependencies(rl_basic_example
  rclcpp
  interface_protocol
)

install(TARGETS
  hold_joint_status
  joint_test_example
  rl_basic_example
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
