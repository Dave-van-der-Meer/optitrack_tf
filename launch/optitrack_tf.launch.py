from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description to start the tf broadcaster node for OptiTrack.
    """
    # Get the path to the broadcast_tf package
    package_dir = get_package_share_directory('optitrack_tf')

    # Load the tracker_name parameter from ./config/params.yaml
    parameters_file_path = Path(
    package_dir,
    'config',
    'params.yaml'
    )

    # Create a Node object to launch the broadcast_tf executable
    node = Node(
        package='optitrack_tf',
        executable='broadcast_tf',
        name='broadcast_tf_node',
        parameters=[parameters_file_path]
    )

    # Create the LaunchDescription object and add the Node object to it
    ld = LaunchDescription()
    ld.add_action(node)

    return ld