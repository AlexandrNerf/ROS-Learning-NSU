import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions
import launch_ros.parameters_type

def generate_launch_description():
    # Путь к файлу URDF
    urdf_file = os.path.join('robot_kud.urdf.xacro')

    # Чтение содержимого файла URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Параметр для описания робота
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(
                Command(['xacro ', str(urdf_file)]), value_type=str
                )}]
        ),

        # GUI Joint State Publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz для визуализации
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join('rviz_config.rviz')]
        )
    ])

