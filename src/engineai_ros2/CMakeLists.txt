cmake_minimum_required(VERSION 3.8)
project(engineai_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interface_protocol REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

## GENREATE URDF FILE
# copy urdf file to share
find_package(mujoco_simulator REQUIRED)  # get mujoco_simulator_PREFIX here
get_filename_component(_mujoco_simulator_share_dir "${mujoco_simulator_DIR}" DIRECTORY)  # → ".../share/mujoco_simulator"
set(ASSET_SRC_ROOT "${_mujoco_simulator_share_dir}/assets/resource/robot/pm_v2/")
set(ASSET_SRC_MESH_DIR "${ASSET_SRC_ROOT}/meshes")
set(ASSET_SRC_URDF      "${ASSET_SRC_ROOT}/urdf/serial_pm_v2.urdf")

set(ASSET_DST_ROOT "${CMAKE_CURRENT_BINARY_DIR}/resource/robot/pm_v2")
set(ASSET_DST_MESH_DIR "${ASSET_DST_ROOT}/meshes")
set(ASSET_DST_URDF_DIR "${ASSET_DST_ROOT}/urdf")
set(ASSET_DST_URDF     "${ASSET_DST_URDF_DIR}/serial_pm_v2.urdf")

# copy urdf fileas ROS-friendly
file(MAKE_DIRECTORY "${ASSET_DST_MESH_DIR}")
file(MAKE_DIRECTORY "${ASSET_DST_URDF_DIR}")
file(READ  "${ASSET_SRC_URDF}" _urdf_txt)
string(REPLACE "package://resource"
               "package://${PROJECT_NAME}/resource"
               _urdf_txt "${_urdf_txt}")
file(WRITE "${ASSET_DST_URDF}" "${_urdf_txt}")

# copy mesh file
add_custom_command(
  OUTPUT "${ASSET_DST_MESH_DIR}/.copied"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${ASSET_DST_MESH_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${ASSET_SRC_MESH_DIR}" "${ASSET_DST_MESH_DIR}"
  COMMAND ${CMAKE_COMMAND} -E touch "${ASSET_DST_MESH_DIR}/.copied"
  DEPENDS "${ASSET_SRC_MESH_DIR}"
  COMMENT "Copy pm_v2 meshes from mujoco_simulator"
)

add_custom_target(target_assets ALL
  DEPENDS "${ASSET_DST_MESH_DIR}/.copied"
)

# install
install(DIRECTORY config launch urdf ${CMAKE_CURRENT_BINARY_DIR}/resource
  DESTINATION "share/${PROJECT_NAME}"
)

## COMPILE EXECUTABLE
add_executable(joint_command_mux src/joint_command_mux_node.cpp)
target_include_directories(joint_command_mux PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(joint_command_mux
  rclcpp
  std_srvs
  interface_protocol)
target_compile_features(joint_command_mux PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS joint_command_mux
  DESTINATION lib/${PROJECT_NAME})

## COMPILE EXECUTABLE
add_executable(engineai_cmd_vel_controller src/engineai_cmd_vel_controller_node.cpp)
target_include_directories(engineai_cmd_vel_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(engineai_cmd_vel_controller
  rclcpp
  geometry_msgs
  interface_protocol)
target_compile_features(engineai_cmd_vel_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS engineai_cmd_vel_controller
  DESTINATION lib/${PROJECT_NAME})

## COMPILE PLUGINS
add_library(
  engineai_hardware_interface
  SHARED
  src/engineai_hardware_interface.cpp
)
target_compile_features(engineai_hardware_interface PUBLIC cxx_std_17)
target_include_directories(engineai_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/engineai_hardware_interface>
)
ament_target_dependencies(engineai_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  interface_protocol
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface engineai_hardware_interface.xml)

# Install
install(
  DIRECTORY include
  DESTINATION include
)
install(TARGETS engineai_hardware_interface
  EXPORT export_engineai_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## TEST
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_engineai_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
