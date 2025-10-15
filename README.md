# ROS2 Assignment 1 - Robot Information Publisher & Transformer

This repository contains ROS 2 packages for Assignment 1, which implements a publisher-subscriber system with coordinate frame transformation.

## Packages

### 1. assignment1_interfaces
Custom ROS 2 message package containing:
- **RobotInfo.msg**: Custom message type with the following fields:
  - `robot_name` (string): Name of the robot
  - `x` (float64): X position coordinate
  - `y` (float64): Y position coordinate
  - `theta` (float64): Orientation angle in degrees
  - `temperature` (float64): Current temperature reading

### 2. Assignment1_safyan
Main package containing two nodes:

#### Node 1: info_local
- **Type**: Publisher
- **Topic**: `robot_info_local`
- **Functionality**: Publishes robot information in the local reference frame
  - Robot name: "robot1"
  - Position: Arbitrary values (x, y, θ) with some random variation
  - Temperature: Simulated temperature reading with variation
- **Publishing Rate**: 1 Hz (every 1 second)

#### Node 2: info_global
- **Type**: Subscriber + Publisher
- **Subscribed Topic**: `robot_info_local`
- **Published Topic**: `robot_info_global`
- **Functionality**:
  - Subscribes to local robot information
  - Transforms position from local to global reference frame
  - Publishes transformed data
  - Robot name and temperature remain unchanged

## Coordinate Frame Transformation

The transformation between local and global frames involves a rotation of **50 degrees**.

**Transformation Equations:**
```
x_global = x_local × cos(50°) - y_local × sin(50°)
y_global = x_local × sin(50°) + y_local × cos(50°)
θ_global = θ_local + 50°
```

**Note**: There is no translation between frames, only rotation.

## Installation & Setup

### Prerequisites
- ROS 2 Foxy installed
- Python 3.8 or higher
- colcon build tool

### Build Instructions

1. Clone the repository:
```bash
git clone git@github.com:SAFYAN-ELE/ROS2-project.git
cd ROS2-project
```

2. Source ROS 2:
```bash
source /opt/ros/foxy/setup.bash
```

3. Build the workspace:
```bash
colcon build
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Running the Nodes

### Terminal 1 - Run info_local node:
```bash
source install/setup.bash
ros2 run Assignment1_safyan info_local
```

### Terminal 2 - Run info_global node:
```bash
source install/setup.bash
ros2 run Assignment1_safyan info_global
```

## Testing

### Check Topics
```bash
ros2 topic list
```
You should see:
- `/robot_info_local`
- `/robot_info_global`

### Echo Topics
```bash
# Terminal 1 - View local frame data
ros2 topic echo /robot_info_local

# Terminal 2 - View global frame data
ros2 topic echo /robot_info_global
```

### Verify Message Type
```bash
ros2 interface show assignment1_interfaces/msg/RobotInfo
```

## Example Output

**info_local output:**
```
[INFO] Publishing: robot=robot1, pos=(5.40, 2.94, 30.00°), temp=23.64°C
```

**info_global output:**
```
[INFO] Transformed: Local(4.57, 3.04, 30.00°) -> Global(0.61, 5.46, 80.00°), temp=25.26°C
```

## Assignment Requirements Checklist

- ✅ Created ROS package named 'Assignment1_safyan'
- ✅ Created custom ROS message (RobotInfo) with robot_name, position (x, y, θ), and temperature
- ✅ Implemented 'info_local' node that publishes robot information
- ✅ Implemented 'info_global' node that subscribes, transforms coordinates, and republishes
- ✅ Applied 50° rotation transformation between local and global frames
- ✅ Robot name and temperature passed through unchanged
- ✅ Successfully tested both nodes

## Author
Safyan

## License
TODO: License declaration
