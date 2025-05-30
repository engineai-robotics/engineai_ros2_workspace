from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get package directory
    package_dir = get_package_share_directory('mujoco_simulator')

    # Create environment variables
    env_vars = [
        SetEnvironmentVariable('PRODUCT', 'pm_v2'),
        SetEnvironmentVariable('MUJOCO_ASSETS_PATH',
                               PathJoinSubstitution([package_dir, 'assets'])),
        SetEnvironmentVariable('LD_LIBRARY_PATH', [
                               '/opt/engineai_robotics_third_party/lib:/opt/ros/humble/lib:', EnvironmentVariable(name='LD_LIBRARY_PATH', default_value='')])
    ]

    # 根据headless参数创建节点配置
    def launch_setup(context, *args, **kwargs):

        # 准备节点参数
        args = []

        # 定义MuJoCo模拟器节点
        mujoco_node = Node(
            package='mujoco_simulator',
            executable='mujoco_simulator',
            name='mujoco_simulator',
            output='screen',
            emulate_tty=True,
            arguments=args,
            parameters=[
                {'use_sim_time': True},
            ]
        )

        return [mujoco_node]

    # 使用OpaqueFunction来获取上下文中的参数值
    mujoco_launch = OpaqueFunction(function=launch_setup)

    # Return launch description
    return LaunchDescription([
        *env_vars,
        mujoco_launch
    ])
