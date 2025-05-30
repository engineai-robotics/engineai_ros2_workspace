from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package path using get_package_share_directory
    package_dir = get_package_share_directory('interface_example')

    # Get product from environment variable or use default 'pm01'
    product = os.environ.get('PRODUCT', 'pm01')
    
    # Build config file path using os.path.join
    config_dir = os.path.join(package_dir, 'config',
                              product, 'rl_basic', 'basic')
    print(config_dir)
    # Ensure config directory exists
    if not os.path.exists(config_dir):
        raise FileNotFoundError(f"Config directory not found: {config_dir}")

    # Append dynamic library path
    ENGINEAI_ROBOTICS_THIRD_PARTY = "/opt/engineai_robotics_third_party"
    os.environ['LD_LIBRARY_PATH'] = os.path.join(
        ENGINEAI_ROBOTICS_THIRD_PARTY, 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', '')

    # Create node
    hardware_node = Node(
        package='interface_example',
        executable='rl_basic_example',
        name='rl_basic_example',
        arguments=[config_dir],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        hardware_node
    ])
