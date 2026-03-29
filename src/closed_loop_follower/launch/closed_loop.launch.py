from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        
        
        
        
        Node(
            package='position_node',
            executable='position_node',
            name='position_node'
        ),
        
        
        Node(
            package='closed_loop_follower',
            executable='closed_loop_follower',
            name='closed_loop_follower',
            parameters=[{
                'tau_x': 1.0,
                'max_speed': 2.0,
                'pixel_to_meter': 0.002,
                'tau_theta': 0.8
            }]
        ),
        
       
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_simulator',
            parameters=[{
                'image_stream_FPS': 30.0,
                'visual_frequency_turtlesim': 62.5
            }]
        ),
        
       
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
    ])