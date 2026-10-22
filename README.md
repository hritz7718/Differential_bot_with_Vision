# DiffBot: ROS 2 Humble Mobile Robot

An educational differential-drive robot project for ROS 2 Humble and Gazebo
Classic. DiffBot has simulated LiDAR, IMU, RGB camera, YOLO object detection,
and Python behaviour nodes.

## What is included

| Package | Purpose |
| --- | --- |
| `my_diffbot_description` | Modular Xacro robot model, sensor frames, Gazebo plugins, RViz configuration |
| `my_diffbot_gazebo` | Gazebo launch file and worlds |
| `my_diffbot_navigation` | IMU monitor, obstacle avoidance, target search/follower |
| `my_diffbot_vision` | YOLO11n RGB-camera detection and annotated-image output |
| `my_diffbot_bringup` | Optional one-command simulation bringup launch |

## Robot interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `/diffbot/cmd_vel` | `geometry_msgs/Twist` | Drive command |
| `/diffbot/odom` | `nav_msgs/Odometry` | Simulated wheel odometry |
| `/diffbot/scan` | `sensor_msgs/LaserScan` | 360-degree LiDAR |
| `/diffbot/imu/data` | `sensor_msgs/Imu` | IMU orientation, angular velocity, acceleration |
| `/diffbot/camera/image_raw` | `sensor_msgs/Image` | RGB camera stream |
| `/diffbot/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration information |
| `/diffbot/yolo/detections` | `vision_msgs/Detection2DArray` | YOLO 2D detections |
| `/diffbot/yolo/annotated_image` | `sensor_msgs/Image` | Camera image with detection labels and boxes |
| `/diffbot/target_pose` | `geometry_msgs/PoseStamped` | Current follow target estimated in `odom` |
| `/diffbot/detected_object_poses` | `geometry_msgs/PoseArray` | Remembered object estimates in `odom` |

The simulated TF tree is:

```text
odom -> base_footprint -> base_link -> camera_link -> camera_optical_frame
                                      -> laser_link
                                      -> imu_link
                                      -> left_wheel_link
                                      -> right_wheel_link
```

## Requirements

- Ubuntu with ROS 2 Humble and Gazebo Classic
- `gazebo_ros`, `robot_state_publisher`, RViz, Xacro
- Python 3 virtual environment at `/home/ragnarhritz/.venvs/diffbot_vision`
  containing `ultralytics`, PyTorch, OpenCV, NumPy 1.x, and Colcon

The YOLO weights are included in:

```text
my_diffbot_vision/models/yolo11n.pt
```

## Build

```bash
cd /home/ragnarhritz/diffbot_ws
source /opt/ros/humble/setup.bash
source /home/ragnarhritz/.venvs/diffbot_vision/bin/activate

colcon build --symlink-install
source install/setup.bash
```

## Run the simulation

Start Gazebo and dynamically spawn DiffBot into the default obstacle world:

```bash
ros2 launch my_diffbot_gazebo gazebo.launch.py
```

For the optional general bringup launch (Gazebo, IMU monitor, YOLO, RViz):

```bash
ros2 launch my_diffbot_bringup full_simulation.launch.py
```

This launch deliberately does **not** start autonomous movement by default.

## Sensor checks

```bash
ros2 topic hz /diffbot/scan
ros2 topic hz /diffbot/imu/data
ros2 topic hz /diffbot/camera/image_raw

ros2 topic echo --once /diffbot/imu/data
rqt_image_view /diffbot/camera/image_raw
```

## YOLO detection

Start the detector after Gazebo is running:

```bash
ros2 run my_diffbot_vision yolo_detector
```

View annotated detections:

```bash
rqt_image_view /diffbot/yolo/annotated_image
ros2 topic echo --once /diffbot/yolo/detections
```

The detector annotates every object YOLO recognizes. Its current follow-class
defaults are configured in `my_diffbot_vision/Yolo_detection.py`. Pretrained
YOLO11n recognizes COCO classes such as `person`, `bottle`, `bowl`, `chair`,
and `cell phone`; realistic Gazebo models work much better than primitives.

## Behaviour nodes

### IMU monitor

```bash
ros2 run my_diffbot_navigation imu_monitor --ros-args -p use_sim_time:=true
```

Prints roll, pitch, yaw, angular velocity, and acceleration in the terminal.

### Basic obstacle avoidance

```bash
ros2 run my_diffbot_navigation obstacle_avoidance --ros-args \
  -p use_sim_time:=true
```

Uses only LiDAR to move forward in open space and turn away from obstacles.

### Target search and follower

```bash
ros2 run my_diffbot_navigation target_follower --ros-args \
  -p use_sim_time:=true
```

The follower is a single controller with these states:

```text
SEARCH: no target detected; move slowly through clear LiDAR space.
FOLLOW: follow configured target classes using camera direction + LiDAR range.
SAFETY: stop for nearby people or unsafe LiDAR readings.
```

It currently follows `bottle` and `bowl`, avoids `person`, and uses LiDAR as
the final collision check. Do **not** run it together with `obstacle_avoidance`
or teleoperation: all of them publish `/diffbot/cmd_vel`.

Tune target classes at runtime, for example:

```bash
ros2 run my_diffbot_navigation target_follower --ros-args \
  -p use_sim_time:=true \
  -p follow_classes:="['bottle', 'bowl']" \
  -p avoid_classes:="['person']"
```

## Worlds

The default simulation world is:

```text
my_diffbot_gazebo/worlds/obstacle_world.world
```

To edit it in plain Gazebo:

```bash
ros2 launch gazebo_ros gazebo.launch.py \
  world:=/home/ragnarhritz/diffbot_ws/src/my_diffbot_gazebo/worlds/obstacle_world.world
```

Use **File → Save World** after adding obstacles. Keep the dynamically spawned
`diffbot` out of the saved world to prevent duplicate robots on normal launch.

## Current limitations and next steps

- Object positions are estimates in the drifting `odom` frame, not fixed map
  coordinates.
- The camera is RGB only. YOLO identifies classes but LiDAR provides distance.
- Search is reactive LiDAR exploration, not systematic coverage.
- The next navigation milestone is SLAM Toolbox, then Nav2 and frontier
  exploration for map-based object positions and navigation.
