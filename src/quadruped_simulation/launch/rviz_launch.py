import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory('quadruped_simulation'), 'urdf', '4leggedRobot-5.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('quadruped_simulation'), 'rviz', 'quadruped_robot.rviz')
    
    return LaunchDescription([
        Node(
            package='quadruped_simulation',
            executable='states_subscriptor.py',
            name='states_subscriptor',
            shell=True
        ),
        # publish robot state from urdf file
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])