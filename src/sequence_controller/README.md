### Description
This package implements a time-based sequence controller that publishes a 5-step cyclic trajectory to the RELbot simulator.

### Inputs
None (open-loop controller)

### Outputs
`/input/motor_cmd`
Type: relbot_msgs/msg/RelbotMotors
The message contains `left_wheel_vel` and `right_wheel_vel` in rad/s

### Run
In a terminal run the following command:
```bash

ros2 launch sequence_controller test_sequence.launch.py

