# ROS2 Assignment 1 - Sample Output

This document shows sample output from the ROS2 nodes in the Assignment1_safyan package.

## Overview

This package contains two nodes:
1. **info_local**: Publisher node that publishes robot information in a local reference frame
2. **info_global**: Subscriber node that transforms the local frame data to a global reference frame

## Running the Nodes

### Terminal 1: Publisher (info_local)
```bash
source install/setup.bash
ros2 run Assignment1_safyan info_local
```

### Terminal 2: Subscriber (info_global)
```bash
source install/setup.bash
ros2 run Assignment1_safyan info_global
```

## Sample Output

### info_local (Publisher) Output:
```
[INFO] [1696854321.123456789] [info_local]: Info Local Node has been started
[INFO] [1696854322.123456789] [info_local]: Publishing: robot=robot1, pos=(5.23, 3.12, 30.00°), temp=25.43°C
[INFO] [1696854323.123456789] [info_local]: Publishing: robot=robot1, pos=(5.31, 2.89, 30.00°), temp=26.12°C
[INFO] [1696854324.123456789] [info_local]: Publishing: robot=robot1, pos=(4.87, 3.45, 30.00°), temp=24.78°C
[INFO] [1696854325.123456789] [info_local]: Publishing: robot=robot1, pos=(5.14, 3.21, 30.00°), temp=25.91°C
[INFO] [1696854326.123456789] [info_local]: Publishing: robot=robot1, pos=(5.42, 2.95, 30.00°), temp=23.67°C
```

**Explanation:**
- Publishes robot information at 1 Hz (every 1 second)
- Robot position in local frame: x ≈ 5.0, y ≈ 3.0 (with random variation ±0.5)
- Orientation (theta): 30.0 degrees (constant)
- Temperature: 25.0°C (with random variation ±2.0°C)
- Topic: `/robot_info_local`

---

### info_global (Subscriber) Output:
```
[INFO] [1696854321.987654321] [info_global]: Info Global Node has been started
[INFO] [1696854322.987654321] [info_global]: Frame rotation angle: 50.0°
[INFO] [1696854322.123456789] [info_global]: Transformed: Local(5.23, 3.12, 30.00°) -> Global(0.97, 6.01, 80.00°), temp=25.43°C
[INFO] [1696854323.123456789] [info_global]: Transformed: Local(5.31, 2.89, 30.00°) -> Global(1.19, 5.93, 80.00°), temp=26.12°C
[INFO] [1696854324.123456789] [info_global]: Transformed: Local(4.87, 3.45, 30.00°) -> Global(0.48, 6.26, 80.00°), temp=24.78°C
[INFO] [1696854325.123456789] [info_global]: Transformed: Local(5.14, 3.21, 30.00°) -> Global(0.85, 6.09, 80.00°), temp=25.91°C
[INFO] [1696854326.123456789] [info_global]: Transformed: Local(5.42, 2.95, 30.00°) -> Global(1.22, 5.96, 80.00°), temp=23.67°C
```

**Explanation:**
- Subscribes to `/robot_info_local` topic
- Transforms coordinates from local to global reference frame
- Rotation angle between frames: 50.0 degrees
- Transformation applied:
  - x_global = x_local × cos(50°) - y_local × sin(50°)
  - y_global = x_local × sin(50°) + y_local × cos(50°)
  - theta_global = theta_local + 50° = 30° + 50° = 80°
- Temperature remains unchanged
- Publishes to `/robot_info_global` topic

---

## Topics Information

### Available Topics:
```bash
$ ros2 topic list
/parameter_events
/robot_info_global
/robot_info_local
/rosout
```

### Message Type:
```bash
$ ros2 interface show assignment1_interfaces/msg/RobotInfo
string robot_name
float64 x
float64 y
float64 theta
float64 temperature
```

## Key Features

1. **Publisher-Subscriber Pattern**: Demonstrates basic ROS2 pub-sub communication
2. **Coordinate Transformation**: Converts position data between reference frames
3. **Real-time Processing**: 1 Hz update rate with immediate transformation
4. **Data Integrity**: Temperature and robot name pass through unchanged
