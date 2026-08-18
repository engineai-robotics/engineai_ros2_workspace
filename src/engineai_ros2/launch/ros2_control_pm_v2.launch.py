# Copyright 2025 Kei Okada
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

robot_description = ParameterValue(
    Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("engineai_ros2"),
            "urdf", "pm_v2.urdf.xacro",
        ]),
    ]),    
    value_type=str,
)

def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start RViz2 automatically with this launch file.",
    )
    joint_command_ros_arg = DeclareLaunchArgument(
        'joint_command_ros', default_value='/hardware/joint_command_ros'
    )
    joint_command_rl_arg = DeclareLaunchArgument(
        'joint_command_rl', default_value='/hardware/joint_command_rl'
    )
    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("engineai_ros2"),
                "config","pm_v2_controllers.yaml",
            ])
        ],
        remappings=[
            ('/hardware/joint_command', LaunchConfiguration('joint_command_ros')),
        ],
        output="both",
    )
    # robot_state_publisher with robot_description
    state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": robot_description
            }
        ],
    )
    mux_node = Node(
        package='engineai_ros2',
        executable='joint_command_mux',
        name='joint_command_mux',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('engineai_ros2'),
                'config', 'pm_v2_joint_command_mux.yaml',
            ])
        ],
        remappings=[
            ('/hardware/joint_command_ros', LaunchConfiguration('joint_command_ros')),
            ('/hardware/joint_command_rl', LaunchConfiguration('joint_command_rl')),
            # The output topic can also be remapped if needed:
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathSubstitution(FindPackageShare("engineai_ros2"))
            / "config/pm_v2.rviz",
        ],
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="log",
        parameters = [
            PathJoinSubstitution([
                FindPackageShare("engineai_ros2"),
                "config",
                "ekf_imu_only.yaml"
            ])
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "robot_localization:=debug"
            ]
        )
    broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            PathSubstitution(FindPackageShare("engineai_ros2"))
            / "config"
            / "pm_v2_controllers.yaml",
        ],
    )
    controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            PathSubstitution(FindPackageShare("engineai_ros2"))
            / "config"
            / "pm_v2_controllers.yaml",
        ],
    )
    cmd_vel_controller_node = Node(
        package='engineai_ros2',
        executable='engineai_cmd_vel_controller',
        name='engineai_cmd_vel_controller',
        output='screen',
    )
    return LaunchDescription(
        [
            gui_arg,
            joint_command_ros_arg,
            joint_command_rl_arg,
            control_node,
            state_publisher_node,
            mux_node,
            ekf_node,
            rviz_node,
            broadcaster_node,
            controller_node,
            cmd_vel_controller_node
        ]
    )
