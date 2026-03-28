### Description
This package implements a closed-loop position controller based on first-order dynamics that uses visual feedback from the moving camera to make the RELbot track a bright object.

### Inputs
`/object_position`
Type: geometry_msgs/msg/Point
Object coordinates from position_node (pixel coordinates)
- x: horizontal pixel coordinate
- y: vertical pixel coordinate
- z: area/size of detected object

`/output/robot_pose`
Type: geometry_msgs/msg/PoseStamped
Current robot pose from simulator
- position.x: robot x position in world coordinates
- orientation.z: robot orientation (theta) in radians

### Outputs
`/input/motor_cmd`
Type: relbot_msgs/msg/RelbotMotors
The message contains left_wheel_vel and right_wheel_vel in rad/s

### Run
In a terminal run the following command:
```bash
ros2 run closed_loop_follower closed_loop_follower --ros-args \
  -p tau:=0.5 \
  -p pixel_to_meter:=0.005 \
  -p img_center:=150.0