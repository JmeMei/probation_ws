# ROS-TCP-Endpoint Runtime Guide
## 1. Overview
[![Watch the video](https://img.youtube.com/vi/c4qOdlP8Z78/0.jpg)](https://youtu.be/c4qOdlP8Z78)

This document describes how to start the full simulation + tooling stack:
- ROS 2 TCP endpoint server
- Unity simulation (external executable)
- Foxglove bridge + Foxglove Studio
- Robot bringup (sensors, MAVROS, navigation)
- Custom scripts: gate_navigator.py and to_guided_mode.py

## 2. Build & Environment Setup
Run once per new terminal:
source install/setup.bash

(Optional) Add to ~/.bashrc for convenience.

## 3. Launch Order (open 4 terminals)
Terminal 1 (start first – Unity connects here):
ros2 run ros_tcp_endpoint default_server_endpoint

Terminal 2 (start Unity executable):
(Windows) Double-click UnitySim.x86_64
(or via WSL path mount if applicable)

Terminal 3 (Foxglove bridge):
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
Then open Foxglove (desktop app or web) and connect (default ws port 8765 unless changed).

Terminal 4 (robot bringup):
ros2 launch my_robot_bringup jamie_robot.launch.xml  
*Note: This launch file starts `to_guided_mode.py` and `gate_navigator.py`.*

## 4. Key Scripts 
### [open gate_navigator.py](src/ROS-TCP-Endpoint/ros_tcp_endpoint/to_guided_mode.py)
- Purpose: Navigate toward a detected gate using its bounding box in the camera feed.
- Typical Inputs: /main_camera/detection/bounding_boxes (vision_msgs/BoundingBoxArray).
- Typical Output Topic: /mavros/setpoint_velocity/cmd_vel_unstamped (geometry_msgs/Twist) for velocity commands.
- Helper Movement : The script defines small readable functions (publish_velocity_command(), sink(), move_forward(), rotate_clockwise(), stop_movement()) so higher-level logic is clearer and easier to modify.

Flow / Logic Overview:
1. Depth Acquisition: Using Foxglove, the desired operating depth was determined manually; the value was then confirmed by echoing /mavros/global_position/rel_alt. That numeric depth is stored as target_depth.
2. Descent Phase: The node periodically compares current rel_alt to target_depth and commands a downward velocity via sink() until the target depth is reached, then stops vertical motion.
3. Search Phase: After stabilizing at depth, it rotates (clockwise) to search for a bounding box whose label_name == "gate" and confidence exceeds the threshold.
4. Tracking / Alignment: Once a gate box is found, it continuously adjusts yaw while moving forward, attempting to center the gate horizontally (x close to 0.5 in normalized image space).
5. Proximity Check: The gate's apparent size (area = w * h in normalized units) is monitored. When area exceeds a configured threshold, the script enters a short forward pass-through phase.
6. Pass-Through Completion: After moving forward for a fixed duration (or once logic deems it passed), motion is stopped and navigation concludes.
7. Readability Benefit: Using the small helper functions keeps the behavioral sections (depth control, search, alignment, pass-through) easy to scan and adjust without digging into low-level Twist construction.

### [open to_guided_mode.py](src/ROS-TCP-Endpoint/ros_tcp_endpoint/to_guided_mode.py)
- Purpose: Automatically switch the vehicle into GUIDED mode on startup.
- Mechanism: Calls /mavros/set_mode (and optionally arming after readiness checks).
- Integration: Launched inside jamie_robot.launch.xml.
- Service discovery used to determine how to change the mode:
```bash
ros2 service list                  # enumerate all services
ros2 service type /mavros/set_mode # confirm service type
ros2 interface show mavros_msgs/srv/SetMode  # inspect request fields (base_mode, custom_mode)
```
(These showed that setting custom_mode: 'GUIDED' with base_mode: 0 is sufficient.)

## 5. Suggested Improvement (Automation)
An improvement could be to implement a PID controller to stabilize the robot's movement as it approaches the gate. Currently, the robot exhibits excessive lateral oscillations (moving left and right) while navigating toward the gate. A PID controller can help smooth out the trajectory by dynamically adjusting the robot's velocity or position setpoints based on the error in its alignment with the gate.

## 6. Useful Commands

### Echo only detections around "gate"
```bash
ros2 topic echo /main_camera/detection/bounding_boxes | grep -A 10 -B 10 "label_name: gate"
```

### Set GUIDED flight mode (MAVROS) (if you don't run my script)
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
```

### Check the mode 
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" 
```

### Pull a single file from a Git branch
```bash
git checkout origin/main -- relative/path/to/file.py
```
# End
