# Advanced Software Development for Robotics - Assignment 1

## Group Information
- **Group Members**: Afeef & Vaishnav
- **Course**: ASDFR 
- **Date**: February 2026

---

## Prerequisites
```bash
# Source your workspace in every new terminal
cd ~/assignment_1_ws
source install/setup.bash

# If you need to rebuild everything
colcon build
source install/setup.bash

Assignment 1.1.1: Camera Input with ROS2
Files Used

    cam2image_vm2ros package (provided)

Bash Commands

Terminal 1 (Host Machine - Outside VM):
bash

python videoserver.py

Terminal 2 (VM):
bash

ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml

Terminal 3 (VM):
bash

ros2 run image_tools showimage

Terminal 4 (VM):
bash

# Check QoS settings
ros2 topic info /image --verbose

# Monitor frame rate
ros2 topic hz /image

Expected Output

    Window shows webcam feed

    Terminal shows publishing messages

    QoS: depth=1, history=keep_last

Assignment 1.1.2: Brightness Detection Node
Files Created

    Package: brightness_node

    Node: brightness_node

Build
bash

cd ~/assignment_1_ws
colcon build --packages-select brightness_node
source install/setup.bash

Bash Commands

Terminal 1 (Host):
bash

python videoserver.py

Terminal 2 (VM):
bash

ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml

Terminal 3 (VM):
bash

ros2 run brightness_node brightness_node

Terminal 4 (VM):
bash

# Monitor light/dark status
ros2 topic echo /light_status

Testing

    Cover webcam → data: false

    Uncover webcam → data: true

Assignment 1.1.3: ROS2 Parameters
Files Modified

    brightness_node/src/brightness_node.cpp

Build
bash

cd ~/assignment_1_ws
colcon build --packages-select brightness_node
source install/setup.bash

Bash Commands

Start with custom threshold:
bash

ros2 run brightness_node brightness_node --ros-args -p brightness_threshold:=150.0

Change threshold at runtime:
bash

# Check current value
ros2 param get /brightness_node brightness_threshold

# Set new values
ros2 param set /brightness_node brightness_threshold 50.0
ros2 param set /brightness_node brightness_threshold 200.0

Assignment 1.1.4: Object Position Indicator
Files Created

    Package: position_node

    Node: position_node

Build
bash

cd ~/assignment_1_ws
colcon build --packages-select position_node
source install/setup.bash

Bash Commands

Terminal 1 (Host):
bash

python videoserver.py

Terminal 2 (VM):
bash

ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml

Terminal 3 (VM):
bash

ros2 run position_node position_node

Terminal 4 (VM):
bash

# Monitor object position
ros2 topic echo /object_position

Testing

    Move light left/right → x changes

    Move light up/down → y changes

    Move closer/further → z (area) changes

    No light → (-1, -1, 0)

Assignment 1.2.1: Sequence Controller with RELbot Simulator
Files Created

    Package: sequence_controller

    Node: sequence_controller

    Launch file: launch/test_sequence.launch.py

Build
bash

cd ~/assignment_1_ws
colcon build --packages-select sequence_controller
source install/setup.bash

Bash Commands

Run everything with launch file:
bash

ros2 launch sequence_controller test_sequence.launch.py

Or run nodes separately:

Terminal 1:
bash

ros2 run relbot_simulator relbot_simulator

Terminal 2:
bash

ros2 run turtlesim turtlesim_node

Terminal 3:
bash

ros2 run sequence_controller sequence_controller

Monitoring

Check nodes:
bash

ros2 node list

Check topics:
bash

ros2 topic list

Monitor motor commands:
bash

ros2 topic echo /input/motor_cmd

Monitor robot position:
bash

ros2 topic echo /output/robot_pose

Visualize:
bash

rqt_graph

Plot position:
bash

ros2 run rqt_plot rqt_plot /output/robot_pose/pose/position/x \
                           /output/robot_pose/pose/position/y \
                           /output/robot_pose/pose/orientation/z


## Assignment 1.2.2: Object Following with Image Processing

### New Package Created
- Package: `object_follower`
- Node: `object_follower`

### Build
```bash
cd ~/assignment_1_ws
colcon build --packages-select object_follower
source install/setup.bash

Files Used

    position_node (from 1.1.4) - detects object position

    object_follower (new) - controls robot to follow object

    relbot_simulator - simulates robot movement

    cam2image_vm2ros - provides camera feed

Bash Commands - All Terminals

Terminal 1 (Host Machine - Outside VM):
bash

# Start video server
python videoserver.py

Terminal 2 (VM):
bash

# Start camera node with correct image dimensions
ros2 run cam2image_vm2ros cam2image --ros-args \
  --params-file src/cam2image_vm2ros/config/cam2image.yaml

Terminal 3 (VM):
bash

# Start object position detector (from 1.1.4)
cd ~/assignment_1_ws
source install/setup.bash
ros2 run position_node position_node

Terminal 4 (VM):
bash

# Start object follower (NEW for 1.2.2)
cd ~/assignment_1_ws
source install/setup.bash
ros2 run object_follower object_follower --ros-args \
  -p image_width:=300.0 \
  -p gain:=0.01 \
  -p base_speed:=0.5

Terminal 5 (VM):
bash

# Start RELbot simulator
cd ~/assignment_1_ws
source install/setup.bash
ros2 run relbot_simulator relbot_simulator

Terminal 6 (VM):
bash

# Start turtlesim visualization
ros2 run turtlesim turtlesim_node
