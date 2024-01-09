from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    return LaunchDescription([
        # Launch the rosbridge server
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),

        # Launch the object detector node
        Node(
            package='object_detector',
            executable='object_detector_node',
            name='object_detector_node'
        ),
    ])
