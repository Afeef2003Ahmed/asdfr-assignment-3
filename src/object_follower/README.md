### Description
This package implements an object following controller that uses position feedback to make the RELbot track a bright object.

### Inputs
`/object_position`
Type: geometry_msgs/msg/Point
Object coordinates from position_node

### Outputs
`/input/motor_cmd`
Type: relbot_msgs/msg/RelbotMotors
The message contains`left_wheel_vel` and `right_wheel_vel` in rad/s

### Run
In a terminal run the following commands:
```bash
ros2 run object_follower object_follower --ros-args \
  -p gain:=0.005 \
  -p base_speed:=0.5 \
  -p image_width:=300.0