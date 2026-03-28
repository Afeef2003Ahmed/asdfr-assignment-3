# follower_launch_122.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Terminal 1: cam2image with config file
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[
                {'depth': 1},
                {'history': 'keep_last'}
            ],
            remappings=[
                ('/image', '/image')
            ]
        ),
        
        # Terminal 2: position_node (no parameters needed based on your command)
        Node(
            package='position_node',  # Your package name
            executable='position_node',
            name='position_node'
        ),
        
        # Terminal 3: object_follower with your parameters
        Node(
            package='object_follower',  # Your package name
            executable='object_follower',
            name='object_follower',
            parameters=[
                {'image_width': 300.0},
                {'gain': 0.015},
                {'base_speed': 0.4},
                {'max_speed': 2.0},
                {'dead_zone': 20.0}
            ]
        ),
        
        # Terminal 4: relbot_simulator
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_sim',
            parameters=[
                {'image_stream_FPS': 30.0}
            ]
        ),
        
        # Terminal 5: turtlesim_node for visualization
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        )
    ])