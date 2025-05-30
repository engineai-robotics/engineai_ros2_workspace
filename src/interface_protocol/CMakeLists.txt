cmake_minimum_required(VERSION 3.8)
project(interface_protocol)

option(GEN_MESSAGE "Generate ROS message files" ON)

find_package(ament_cmake REQUIRED)

if(GEN_MESSAGE)
    find_package(rosidl_default_generators REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    # 使用 GLOB 查找文件
    file(GLOB MSG_FILES "msg/*.msg")
    file(GLOB SRV_FILES "srv/*.srv")

    # 转换为相对路径
    foreach(MSG_FILE ${MSG_FILES})
    file(RELATIVE_PATH REL_MSG_FILE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSG_FILE}")
    list(APPEND REL_MSG_FILES "${REL_MSG_FILE}")
    endforeach()

    foreach(SRV_FILE ${SRV_FILES})
    file(RELATIVE_PATH REL_SRV_FILE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRV_FILE}")
    list(APPEND REL_SRV_FILES "${REL_SRV_FILE}")
    endforeach()

    # 打印找到的文件，便于调试
    message(STATUS "Found message files: ${REL_MSG_FILES}")
    message(STATUS "Found service files: ${REL_SRV_FILES}")

    rosidl_generate_interfaces(${PROJECT_NAME}
    ${REL_MSG_FILES}
    ${REL_SRV_FILES}
        DEPENDENCIES
        std_msgs
        std_srvs
        geometry_msgs
    )
endif()

ament_package() 