### Description
This package implements a brightness detection node that subscribes to camera images and publishes whether the scene is light or dark based on average pixel brightness.

### Inputs
`/image`
Type: sensor_msgs/msg/Image
Camera image stream in bgr8 format

### Outputs
`/light_status`
Type: std_msgs/msg/Bool
True if average brightness > threshold, False otherwise

### Run
In a terminal run the following commands:
```bash
ros2 run brightness_node brightness_node