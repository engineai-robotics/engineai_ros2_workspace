# CMakeLists.txt for the MuJoCo simulate library in a ROS2 package
# This matches the LCM version's configuration for third-party libraries

# For lodepng dependency
include(FetchContent)
set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

# Fetch lodepng dependency
if(NOT TARGET lodepng)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif()
endif()

# Platform UI adapter library
add_library(platform_ui_adapter OBJECT
  ${CMAKE_CURRENT_SOURCE_DIR}/glfw_adapter.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/glfw_dispatch.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/platform_ui_adapter.cc
)

target_include_directories(platform_ui_adapter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${ENGINEAI_ROBOTICS_THIRD_PARTY}/include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(platform_ui_adapter PUBLIC mujoco)

# Main simulate library
add_library(libsimulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)

# Set output name to "simulate"
set_target_properties(libsimulate PROPERTIES OUTPUT_NAME simulate)

target_sources(libsimulate
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/simulate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/array_safety.h
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/simulate.cc
)

target_include_directories(libsimulate PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${lodepng_SOURCE_DIR}
  ${ENGINEAI_ROBOTICS_THIRD_PARTY}/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Set library paths from parent CMakeLists.txt
set(MUJOCO_LIB "${ENGINEAI_ROBOTICS_THIRD_PARTY}/lib/libmujoco.so")

# Link against dependencies
target_link_libraries(libsimulate PUBLIC
  lodepng
  platform_ui_adapter
  ${MUJOCO_LIB}
  glfw
  ${CMAKE_DL_LIBS}
)

# Make sure to export the library with position independent code
set_target_properties(libsimulate PROPERTIES POSITION_INDEPENDENT_CODE ON)
