from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[{
                'width': 300,
                'height': 200,
                'socket_ip': '172.24.0.1',  # Change to your host IP
                'socket_port': 9999,
                'remote_mode': True,
                'show_camera': True,
                'history': 'keep_last',
                'depth': 10,
                'rotate': False
            }]
        ),
        
        
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
                'tau': 1.0,
                'max_speed': 2.0,
                'pixel_to_meter': 0.002,
                'img_center': 150.0
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