# ─────────────────────────────────────────────────
# Assignment 1.1.1
# ─────────────────────────────────────────────────
```bash

Terminal 1 (Host Machine):

python videoserver.py

Terminal 2 (VM):

ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml

Terminal 3 (VM):

ros2 run image_tools showimage

Terminal 4 (VM):

# Check QoS 
ros2 topic info /image --verbose

# Monitor frame rate
ros2 topic hz /image

```
# ─────────────────────────────────────────────────
# Assignment 1.1.2
# ─────────────────────────────────────────────────
``` bash 


# Terminal 1 (Host)
python videoserver.py

bash

# Terminal 2 (VM)
ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml

bash

# Terminal 3 (VM)
ros2 run brightness_node brightness_node

# Terminal 4 (VM)
ros2 topic echo /light_status

# Location in code: brightness_node.cpp in brightness_node package

```

# ─────────────────────────────────────────────────
# Assignment 1.1.3
# ─────────────────────────────────────────────────

``` bash 

# Run with custom threshold

ros2 run brightness_node brightness_node --ros-args -p brightness_threshold:=150.0

# Change threshold at runtime

ros2 param get /brightness_node brightness_threshold
ros2 param set /brightness_node brightness_threshold 50.0
ros2 param set /brightness_node brightness_threshold 200.0

# Location in code: brightness_node.cpp (parameter declaration)

```
# ─────────────────────────────────────────────────
# Assignment 1.1.4
# ─────────────────────────────────────────────────

```bash 

# Terminal 1 (Host)
python videoserver.py

# Terminal 2 (VM)
ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml

# Terminal 3 (VM)
ros2 run position_node position_node

# Terminal 4 (VM)
ros2 topic echo /object_position

# Location in code: position_node.cpp in position_node package

```
# ─────────────────────────────────────────────────
# Assignment 1.2.1
# ─────────────────────────────────────────────────

``` bash

# Run with launch file

ros2 launch sequence_controller test_sequence.launch.py

# Or to run nodes separately:

# Terminal 1
ros2 run relbot_simulator relbot_simulator

# Terminal 2
ros2 run turtlesim turtlesim_node

# Terminal 3
ros2 run sequence_controller sequence_controller


# Location in code: sequence_controller.cpp in sequence_controller package

```
# ─────────────────────────────────────────────────
# Assignment 1.2.2
# ─────────────────────────────────────────────────

``` bash

# Terminal 1 (Host)
python videoserver.py

# Terminal 2 (VM)
ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml


# Terminal 3 (VM)
ros2 run position_node position_node


# Terminal 4 (VM)
ros2 run object_follower object_follower --ros-args \
  -p image_width:=300.0 \
  -p gain:=0.015 \
  -p base_speed:=0.4


# Terminal 5 (VM)
ros2 run relbot_simulator relbot_simulator


# Terminal 6 (VM)
ros2 run turtlesim turtlesim_node

# Location in code: object_follower.cpp in object_follower package

```

# ─────────────────────────────────────────────────
# Assignment 1.2.3
# ─────────────────────────────────────────────────

```bash
# Terminal 1 (Host)
python videoserver.py

# Terminal 2 (VM)
ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml

# Terminal 3 (VM)
ros2 run position_node position_node

# Terminal 4 (VM)
ros2 run closed_loop_follower closed_loop_follower --ros-args \
  -p tau:=0.2 \
  -p pixel_to_meter:=0.01 \
  -p img_center:=150.0

# Terminal 5 (VM)
ros2 run relbot_simulator relbot_simulator

# Terminal 6 (VM)
ros2 run turtlesim turtlesim_node

# Location in code: closed_loop_follower.cpp in closed_loop_follower package