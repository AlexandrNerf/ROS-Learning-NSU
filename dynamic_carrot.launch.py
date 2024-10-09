import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    print("1000")
    dir_rot = LaunchConfiguration('direction_rotation')
    direction_rotation = DeclareLaunchArgument(
        'direction_rotation',
        default_value='1'
    )
    rad = LaunchConfiguration('radius')
    radius = DeclareLaunchArgument(
        'radius',
        default_value='10'
    )
    print(radius)
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('learning_tf2_py'), 'launch'),
            '/tf2_tutor.launch.py']),
       launch_arguments={'target_frame': 'carrot1'}.items(),
       )

    return LaunchDescription([
        demo_nodes,
        #direction_rotation,
        radius,
        direction_rotation,
        Node(
            package='learning_tf2_py',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
            arguments=[dir_rot, rad],
            #arguments = [direction_rotation, radius]
        ),

    ])
