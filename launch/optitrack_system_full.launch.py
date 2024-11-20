"""
Launch file to start the OptiTrack system for use with ROS 2.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Launch description to start the VRPN_MoCap package and the OptiTrack_TF package.
    """
    server_arg = LaunchConfiguration('server', default='192.168.88.115')

    vrpn_mocap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('optitrack_tf'), '/launch/vrpn_mocap_client.launch.py'
        ]),
        launch_arguments={
            'server': server_arg
        }.items()
    )

    optitrack_tf = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('optitrack_tf'), 'launch'),
         '/optitrack_tf.launch.py']),
      )

    return LaunchDescription([
        vrpn_mocap,
        optitrack_tf,
    ])
