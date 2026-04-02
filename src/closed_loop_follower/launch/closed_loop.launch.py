"""
launch/assignment_1_2_3.launch.py

Assignment 1.2.3 – Closed-loop control of the RELbot Simulator.

Node graph (see Section 6.6.1 of the RELbot Manual for notation):

  cam2image ──/image──► relbot_simulator ──/output/moving_camera──► position_node
                              │                                           │
                              │ /output/robot_pose                        │ /object_position
                              │                                           ▼
                              │                              closed_loop_controller
                              │                                           │ /input/twist
                              │                                           ▼
                              └──────────────────────────────────── relbot_adapter
                                         /input/motor_cmd                │
                              ◄──────────────────────────────────────────┘

Parameters are above the nodes; remappings / topic connections shown inline.

Note: cam2image cannot be started inside this launch file when using a VM/WSL
webcam-over-UDP setup (remote_mode).  Start it separately:
  ros2 run image_tools cam2image --ros-args -p depth:=1 -p history:=keep_last
  (or with remote_mode for VM setups, see Chapter 7 of the RELbot Manual)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('closed_loop_follower')
    params_file = os.path.join(pkg_share, 'config', 'controller_params.yaml')


    # ------------------------------------------------------------------
    # RELbot Simulator — takes /image, publishes /output/moving_camera
    # ------------------------------------------------------------------
    relbot_simulator = Node(
        package='relbot_simulator',
        executable='relbot_simulator',
        name='relbot_simulator',
        parameters=[
            {'throttle_rate': 5000.0},
            {'image_stream_FPS': 30.0},
        ],
    )

    # ------------------------------------------------------------------
    # position_node — detects bright object in /output/moving_camera,
    #                 publishes pixel CoG on /object_position
    # ------------------------------------------------------------------
    position_node = Node(
        package='position_node',
        executable='position_node',
        name='position_node',
        parameters=[params_file],
    )

    # ------------------------------------------------------------------
    # closed_loop_controller_node — subscribes to /object_position,
    #   computes first-order control law, publishes Twist on /input/twist
    # ------------------------------------------------------------------
    closed_loop_follower = Node(
        package='closed_loop_follower',
        executable='closed_loop_follower',
        name='closed_loop_follower',
        parameters=[params_file],
    )

    # ------------------------------------------------------------------
    # relbot_adapter — converts Twist to RelbotMotors motor_cmd,
    #   applies velocity limits, forwards to simulator
    # ------------------------------------------------------------------
    relbot_adapter = Node(
        package='relbot_adapter',
        executable='relbot_adapter',
        name='relbot_adapter',
        parameters=[
            {'use_twist_cmd': True},
            {'max_speed_mps': 0.25},
            {'max_speed_rads': 5.0},
            {'robotmode': 'sim'},
        ],
        remappings=[
            # adapter output → simulator input
            ('/output/motor_cmd', '/input/motor_cmd'),
        ],
    )

    return LaunchDescription([
        relbot_simulator,
        position_node,
        closed_loop_follower,
        relbot_adapter,
    ])
