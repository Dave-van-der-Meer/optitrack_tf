"""Launch file for the VRPN Mocap client node."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description for the VRPN Mocap client node."""
    # Declare arguments
    server_arg = DeclareLaunchArgument(
        'server',
        default_value='localhost',
        description='The server address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3883',
        description='The server port'
    )

    package_dir = get_package_share_directory('vrpn_mocap')
    parameters_file_path = Path(package_dir, 'config', 'client.yaml')

    # Node configuration
    vrpn_client_node = Node(
        package='vrpn_mocap',
        namespace='vrpn_mocap',
        executable='client_node',
        name='vrpn_mocap_client_node',
        parameters=[
            parameters_file_path,
            {'server': LaunchConfiguration('server')},
            {'port': LaunchConfiguration('port')}
        ]
    )

    return LaunchDescription([
        server_arg,
        port_arg,
        vrpn_client_node
    ])
