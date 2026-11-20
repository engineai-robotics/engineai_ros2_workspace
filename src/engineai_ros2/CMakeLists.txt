cmake_minimum_required(VERSION 3.22)
project(engineai_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(mujoco_simulator REQUIRED)
get_filename_component(_mujoco_share "${mujoco_simulator_DIR}" DIRECTORY)
set(ASSET_SRC "${_mujoco_share}/assets/resource/robot/pm_v2")
set(ASSET_DST "${CMAKE_CURRENT_BINARY_DIR}/resource/robot/pm_v2")

file(MAKE_DIRECTORY "${ASSET_DST}/meshes" "${ASSET_DST}/urdf")
file(READ  "${ASSET_SRC}/urdf/serial_pm_v2.urdf" _urdf)
string(REPLACE "package://resource" "package://${PROJECT_NAME}/resource" _urdf "${_urdf}")
file(WRITE "${ASSET_DST}/urdf/serial_pm_v2.urdf" "${_urdf}")

add_custom_command(
  OUTPUT  "${ASSET_DST}/meshes/.copied"
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${ASSET_SRC}/meshes" "${ASSET_DST}/meshes"
  COMMAND ${CMAKE_COMMAND} -E touch "${ASSET_DST}/meshes/.copied"
  DEPENDS "${ASSET_SRC}/meshes"
  COMMENT "Copy pm_v2 meshes from mujoco_simulator"
)
add_custom_target(copy_assets ALL DEPENDS "${ASSET_DST}/meshes/.copied")

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/resource
  DESTINATION "share/${PROJECT_NAME}")

ament_auto_add_library(engineai_hardware_interface SHARED
  src/engineai_hardware_interface.cpp
)
target_compile_definitions(engineai_hardware_interface PRIVATE "PLUGINLIB_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(hardware_interface engineai_hardware_interface.xml)

install(FILES engineai_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME})

ament_auto_add_executable(engineai_cmd_vel_controller
  src/engineai_cmd_vel_controller_node.cpp
)

ament_auto_package(INSTALL_TO_SHARE config launch urdf)
