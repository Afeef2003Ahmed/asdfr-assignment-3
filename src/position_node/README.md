
### Description
This package implements an object position detector that finds bright objects in camera images and calculates their center of mass coordinates.

### Inputs
`/image`
Type: sensor_msgs/msg/Image
Camera image stream in bgr8 format

### Outputs
`/object_position`
Type: geometry_msgs/msg/Point
- `x`: Horizontal pixel coordinate of object center
- `y`: Vertical pixel coordinate of object center
- `z`: Number of detected bright pixels (area/size)
- If no object detected: `(-1, -1, 0)`

### Run
In a terminal run the following commands:
```bash
ros2 run position_node position_node