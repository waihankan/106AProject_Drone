# Tello Hand Gesture Control

Control a DJI Tello drone using **hand gestures** detected by an external camera. Move your fist like a joystick - intuitive and fun!

Built with ROS2 Humble, MediaPipe, and [tello-ros2](https://github.com/tentone/tello-ros2).

## How It Works

```
External Camera → MediaPipe Hand Detection → Position Mapping → Tello Drone
```

- **✊ Make a fist** = Control active
- **Move your arm** = Drone follows (left/right/up/down)
- **Distance from center** = Speed
- **Return to center** = Hover

## Quick Start

```bash
# 1. Build
docker compose build
docker compose up -d
docker exec -it ros2_container bash
colcon build --symlink-install

# 2. Connect to Tello WiFi, then run
ros2 run tello tello

# 3. Implement hand detection (see docs/GETTING_STARTED.md)
```

## Control Gestures

| Gesture | Action |
|---------|--------|
| 👍 Thumbs Up | Takeoff |
| 👎 Thumbs Down | Land |
| ✊ Fist + Move Arm | Control drone position |
| ✋ Open Palm | Emergency stop |
| SPACEBAR (keyboard) | Emergency override |

## Project Structure

```
ros_workspace/src/
├── tello/                   # Drone driver (existing)
├── tello_msg/               # Messages (existing)
├── tello_control/           # Keyboard backup (existing)
│
├── tello_interfaces/        # Custom messages (created)
├── tello_vision/            # Hand detection (to implement)
├── tello_gesture_control/   # Position control (to implement)
└── tello_safety/            # Emergency monitor (to implement)
```

## Documentation

- **[docs/GETTING_STARTED.md](docs/GETTING_STARTED.md)** - Setup and development phases
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** - System design and technical details
- **[docs/GESTURE_CONTROL.md](docs/GESTURE_CONTROL.md)** - Implementation code and examples

## Prerequisites

- Docker
- DJI Tello drone
- USB webcam (Logitech C920 or similar)
- Clear 3m x 3m flying space

## Key Features

- **Position-based control** - Fist position = drone direction
- **Speed scaling** - Distance from center = speed
- **Safety features** - Dead man's switch, keyboard override, battery monitor
- **Real-time** - <100ms latency, 30+ FPS
- **Intuitive** - No complex gestures to memorize

## Development Status

- ✅ ROS2 workspace configured
- ✅ Tello driver integrated
- ✅ Package structure created
- ⏳ Hand detection (to implement)
- ⏳ Position controller (to implement)
- ⏳ Safety monitor (to implement)

## Safety

- **Keyboard SPACEBAR** - Immediate emergency stop
- **Open palm gesture** - Controlled emergency land
- **Dead man's switch** - Hover if hand not detected (2s)
- **Battery monitor** - Auto-land at 20%
- **Geofencing** - Max height and distance limits

## Dependencies

Automatically installed via Dockerfile:
- mediapipe - Hand detection
- opencv-python - Image processing
- pynput - Keyboard monitoring
- djitellopy - Tello driver
- ROS2 Humble

## Getting Help

1. Read [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md)
2. Check [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for system design
3. Review implementation code in [docs/GESTURE_CONTROL.md](docs/GESTURE_CONTROL.md)

## References

- [tello-ros2](https://github.com/tentone/tello-ros2) - Base driver
- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands) - Hand tracking
- [ROS2 Humble](https://docs.ros.org/en/humble/) - Robot framework
