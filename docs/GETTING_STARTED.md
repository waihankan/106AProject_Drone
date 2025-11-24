# Getting Started

## Quick Setup

### 1. Build Workspace

```bash
# Start container
docker compose up -d
docker exec -it ros2_container bash

# Install dependencies (if not already in image)
pip3 install mediapipe opencv-contrib-python pynput

# Build workspace
cd /root/ros_workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Set Up Camera

```bash
# Install USB camera node (if needed)
sudo apt install ros-humble-usb-cam

# Test camera
ros2 run usb_cam usb_cam_node_exe

# View feed
ros2 run rqt_image_view rqt_image_view
```

### 3. Test Tello Connection

```bash
# Connect to Tello WiFi network, then:
ros2 run tello tello --ros-args -p tello_ip:=192.168.10.1

# In another terminal, check status:
ros2 topic echo /status
```

## Development Phases

### Phase 1: Implement Hand Detection

**File**: `ros_workspace/src/tello_vision/tello_vision/hand_detector.py`

**What to implement**:
- Subscribe to `/camera/image_raw`
- Use MediaPipe Hands for detection
- Recognize gestures (fist, thumbs, palm)
- Publish hand landmarks and gesture state
- Publish debug image with overlay

**Test**:
```bash
# Terminal 1: Camera
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Hand detector
ros2 run tello_vision hand_detector

# Terminal 3: Visualize
ros2 run rqt_image_view rqt_image_view /gesture_debug

# Terminal 4: Check gestures
ros2 topic echo /gesture_state
```

### Phase 2: Implement Position Control

**File**: `ros_workspace/src/tello_gesture_control/tello_gesture_control/gesture_controller.py`

**What to implement**:
- Subscribe to `/gesture_state` and `/fist_position`
- Map fist position to velocity (use PositionController from GESTURE_CONTROL.md)
- Add smoothing and deadzone logic
- Subscribe to `/tello/status` for safety checks
- Publish to `/tello/control`

**Test (without flying)**:
```bash
# All from Phase 1, plus:
ros2 run tello_gesture_control gesture_controller --ros-args -p test_mode:=true

# Monitor commands
ros2 topic echo /tello/control
```

### Phase 3: Implement Safety Monitor

**File**: `ros_workspace/src/tello_safety/tello_safety/safety_monitor.py`

**What to implement**:
- Monitor keyboard (pynput) for SPACEBAR
- Subscribe to `/tello/battery`
- Publish to `/emergency_stop` on SPACEBAR or low battery
- Add timeout checking

**Test**:
```bash
ros2 run tello_safety safety_monitor

# Press SPACEBAR and check:
ros2 topic echo /emergency_stop
```

### Phase 4: Full Integration Testing

```bash
# Terminal 1: Tello driver
ros2 run tello tello

# Terminal 2: Camera
ros2 run usb_cam usb_cam_node_exe

# Terminal 3: Hand detection
ros2 run tello_vision hand_detector

# Terminal 4: Gesture control
ros2 run tello_gesture_control gesture_controller

# Terminal 5: Safety monitor
ros2 run tello_safety safety_monitor

# Terminal 6: Visualize
ros2 run rqt_image_view rqt_image_view /gesture_debug
```

## Control Instructions

### Basic Flow
1. Show **👍 Thumbs Up** (hold 2s) → Drone takes off
2. Make **✊ Fist** → Control activated
3. **Move your arm**:
   - Left → Drone flies left
   - Right → Drone flies right
   - Up → Drone ascends
   - Down → Drone descends
   - Center → Drone hovers
4. Show **👎 Thumbs Down** → Drone lands

### Emergency Stop
- **✋ Open Palm** → Controlled emergency land
- **SPACEBAR** (keyboard) → Immediate stop

## Safety Checklist

Before flying:
- [ ] Battery > 50%
- [ ] Clear 3m x 3m space
- [ ] Camera has clear view
- [ ] Hand detection working (check /gesture_debug)
- [ ] Keyboard accessible for emergency
- [ ] Spotter present

## Troubleshooting

### Hand Not Detected
- Check lighting (bright, even)
- Stand 1.5-2m from camera
- Ensure hand is fully visible

### Jerky Control
- Move arm slower and smoother
- Increase smoothing_alpha in control_params.yaml
- Practice without flying first

### Drone Drifts
- Return fist to exact center of frame
- Increase deadzone_radius in config

## Tips

1. **Practice without flying** - test hand detection first
2. **Start slow** - small arm movements initially
3. **Use the deadzone** - return to center to hover
4. **Smooth movements** - no jerking or sudden moves
5. **Stay visible** - keep hand in camera view

## Next Steps

1. Review [ARCHITECTURE.md](ARCHITECTURE.md) for system design
2. Review [GESTURE_CONTROL.md](GESTURE_CONTROL.md) for implementation code
3. Start with Phase 1 (hand detection)
4. Test each phase independently
5. Integrate and test with drone

Good luck! 🚁✊
