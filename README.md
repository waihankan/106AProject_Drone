# Tello ROS2 Hand Gesture Control

ROS2 Humble workspace for controlling DJI Tello drones using **hand gestures** detected by an external camera.

Built on [tello-ros2](https://github.com/tentone/tello-ros2) with MediaPipe for hand detection.

## Prerequisites

- Docker
- DJI Tello drone

## Quick Start

### 1. Build and Start Container

```bash
docker compose build
docker compose up -d
docker exec -it ros2_container bash
```

### 2. Build Workspace (First Time Only)

The SLAM packages are automatically excluded via COLCON_IGNORE on container start.

```bash
cd /root/ros_workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Note: Workspace auto-sources on interactive shell sessions.

### 3. Connect to Tello

- Turn on your Tello drone
- Connect your computer's WiFi to the Tello network (usually `TELLO-XXXXXX`)

### 4. Launch Tello Node

```bash
# Source the workspace (add to .bashrc to make permanent)
source /root/ros_workspace/install/setup.bash

# Launch the tello node
ros2 run tello tello --ros-args -p tello_ip:=192.168.10.1
```

### 5. Test in Another Terminal

```bash
docker exec -it ros2_container bash
source /root/ros_workspace/install/setup.bash

# List topics
ros2 topic list

# View camera (ensure safe space first!)
ros2 topic echo /image_raw

# Takeoff (BE CAREFUL!)
ros2 topic pub --once /takeoff std_msgs/msg/Empty

# Land
ros2 topic pub --once /land std_msgs/msg/Empty
```

### 6. Keyboard Control (Optional)

```bash
ros2 run tello_control tello_control
```

**Controls:** T=takeoff, L=land, WASD=move, Arrows=rotate, F=flip, E=emergency

## Project Structure

```
.
├── Dockerfile              # ROS2 Humble container setup
├── docker-compose.yml      # Container configuration
└── ros_workspace/
    ├── src/
    │   ├── tello/          # Main tello driver
    │   ├── tello_control/  # Keyboard control package
    │   ├── tello_msg/      # Custom message definitions
    │   └── tello-ros2/     # Original git repository
    └── requirements.txt    # Python dependencies
```

## Available Topics

| Topic | Type | Description |
|-------|------|-------------|
| /image_raw | sensor_msgs/Image | Camera feed (30hz) |
| /camera_info | sensor_msgs/CameraInfo | Camera info (2hz) |
| /status | tello_msg/TelloStatus | Drone status (2hz) |
| /imu | sensor_msgs/Imu | IMU data (10hz) |
| /battery | sensor_msgs/BatteryState | Battery status (2hz) |
| /odom | nav_msgs/Odometry | Odometry (10hz) |

## Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| /takeoff | std_msgs/Empty | Takeoff command |
| /land | std_msgs/Empty | Land command |
| /emergency | std_msgs/Empty | Emergency stop (shuts motors) |
| /control | geometry_msgs/Twist | Analog control (-100 to 100) |
| /flip | std_msgs/String | Flip direction (r/l/f/b) |

## Project Structure

```
ros_workspace/src/
├── tello/                   # Tello drone driver
├── tello_control/           # Keyboard control (backup)
├── tello_msg/               # Custom messages
├── tello_interfaces/        # Custom message definitions
├── tello_vision/            # Hand detection (MediaPipe)
├── tello_gesture_control/   # Gesture → drone control
└── tello_safety/            # Safety monitor (keyboard emergency stop)
```

## Hand Gesture Control

See [GESTURE_CONTROL.md](docs/GESTURE_CONTROL.md) for detailed architecture and implementation guide.

**Quick Overview:**
- External camera detects hand position using MediaPipe
- **✊ Fist position controls drone like a joystick** - move your arm to control direction
- Intuitive: fist center = hover, fist left = fly left, fist up = ascend
- Safety features: dead man's switch, keyboard override, battery monitor

**Control Method:**
- 👍 **Thumbs Up** → Takeoff
- 👎 **Thumbs Down** → Land
- ✊ **Make Fist** → Control active (move arm to control drone)
  - Move fist **left** → Drone flies **left**
  - Move fist **right** → Drone flies **right**
  - Move fist **up** → Drone **ascends**
  - Move fist **down** → Drone **descends**
  - Fist at **center** → Drone **hovers**
  - Distance from center = speed
- ✋ **Open Palm** → Emergency Stop
- **SPACEBAR** → Keyboard emergency override

## References

- [Gesture Control Guide](docs/GESTURE_CONTROL.md) - Complete implementation guide
- [tello-ros2 GitHub](https://github.com/tentone/tello-ros2)
- [DJI Tello SDK](https://github.com/dji-sdk/Tello-Python)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
