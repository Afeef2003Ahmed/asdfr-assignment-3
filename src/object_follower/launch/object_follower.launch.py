from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([


        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
      
        Node(
            package='object_follower',
            executable='object_follower',
            name='object_follower',
            output='screen',
            parameters=[
                {'image_width': 300.0},
                {'gain': 0.015},
                {'base_speed': 0.4}
            ]
        ),

     
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_simulator',
            output='screen'
        ),

        
        Node(
            package='position_node',
            executable='position_node',
            name='position_node',
            output='screen'
        ),

    ])