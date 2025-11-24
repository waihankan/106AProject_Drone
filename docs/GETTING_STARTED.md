# Getting Started with Hand Gesture Control

## Quick Setup

### 1. Build the Updated Workspace

```bash
# Start container
docker compose up -d
docker exec -it ros2_container bash

# Install new dependencies
pip3 install mediapipe opencv-contrib-python

# Build workspace (includes new packages)
cd /root/ros_workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Test Basic Tello Connection

```bash
# Terminal 1: Connect to Tello WiFi, then start driver
ros2 run tello tello --ros-args -p tello_ip:=192.168.10.1

# Terminal 2: Check status
ros2 topic echo /status
```

### 3. Set Up Camera

```bash
# Install USB camera node
sudo apt install ros-humble-usb-cam

# Test camera
ros2 run usb_cam usb_cam_node_exe

# View camera feed
ros2 run rqt_image_view rqt_image_view
```

### 4. Test Hand Detection (No Drone)

```bash
# Terminal 1: Start camera
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Start hand detector
ros2 run tello_vision hand_detector

# Terminal 3: Visualize detections
ros2 run rqt_image_view rqt_image_view /gesture_debug

# Terminal 4: Monitor recognized gestures
ros2 topic echo /gesture_command
```

### 5. Test Gesture Control (No Drone Flying)

```bash
# Start gesture controller in test mode
ros2 run tello_gesture_control gesture_controller --ros-args -p test_mode:=true

# Check what commands would be sent
ros2 topic echo /tello/control
```

### 6. Full Integration Test

```bash
# Terminal 1: Tello driver
ros2 run tello tello

# Terminal 2: Camera
ros2 run usb_cam usb_cam_node_exe

# Terminal 3: Hand detection
ros2 run tello_vision hand_detector

# Terminal 4: Gesture control
ros2 run tello_gesture_control gesture_controller

# Now use hand gestures to control!
```

## Next Development Steps

### Phase 1: Hand Detection Node (Current)
**File**: `ros_workspace/src/tello_vision/tello_vision/hand_detector.py`

Implement:
- Subscribe to `/camera/image_raw`
- Use MediaPipe Hands for detection
- Publish hand landmarks
- Publish annotated image to `/gesture_debug`
- Recognize basic gestures
- Publish to `/gesture_command`

**Test**: Hand detection works reliably in different lighting

### Phase 2: Gesture Controller (Current)
**File**: `ros_workspace/src/tello_gesture_control/tello_gesture_control/gesture_controller.py`

Implement:
- Subscribe to `/gesture_command`
- Subscribe to `/tello/status` for safety checks
- Map gestures to Twist messages
- Add smoothing/filtering
- Publish to `/tello/control`
- Emergency stop logic

**Test**: Commands are smooth and responsive

### Phase 3: Safety Features
Add to gesture controller:
- Dead man's switch (require hand visible)
- Battery monitor (auto-land <20%)
- Timeout handler (hover if no command for 2s)
- Emergency gesture (instant land)
- Geofencing (height/distance limits)

**Test**: All safety features work correctly

### Phase 4: Fine Tuning
- Adjust gesture sensitivity
- Tune speed parameters
- Test different camera angles
- Optimize for your lighting conditions
- Add more gestures if needed

## Project Structure

```
ros2/
├── README.md                      # Main documentation
├── GESTURE_CONTROL.md             # Detailed architecture guide
├── GETTING_STARTED.md             # This file (quick start)
├── ARCHITECTURE.md                # System design (advanced CV features)
│
├── docker-compose.yml             # Container setup
├── Dockerfile                     # Image configuration
│
└── ros_workspace/
    ├── requirements.txt           # Python dependencies (updated)
    │
    └── src/
        ├── tello/                 # ✓ Drone driver (existing)
        ├── tello_msg/             # ✓ Messages (existing)
        ├── tello_control/         # ✓ Keyboard control (existing)
        │
        ├── tello_interfaces/      # ✓ Created (needs messages)
        ├── tello_vision/          # ✓ Created (needs implementation)
        └── tello_gesture_control/ # ✓ Created (needs implementation)
```

## Development Workflow

### Daily Development Loop

```bash
# 1. Make changes to Python files
vim ros_workspace/src/tello_vision/tello_vision/hand_detector.py

# 2. No rebuild needed (ament_python with --symlink-install)
# Just restart the node

# 3. Test
ros2 run tello_vision hand_detector

# 4. Check output
ros2 topic echo /gesture_command

# 5. Iterate!
```

### When to Rebuild

Only rebuild when you:
- Add new ROS package
- Modify package.xml
- Modify setup.py
- Add new message definitions (.msg files)

```bash
colcon build --symlink-install --packages-select tello_vision
```

## Camera Recommendations

### Tested Cameras
- Logitech C920/C922/C930e (recommended)
- Built-in laptop camera (works, but limited FOV)
- Any USB webcam with 720p+ resolution

### Camera Placement
```
      [Ceiling Mount - Best]
            ↓
    ───────────────────
    │                 │
    │  Flying Space   │  <-- Drone flies here
    │                 │
    ───────────────────
         [User]

OR

    [Tripod - Good]
         │
         │  1.5m height
         │
      [User] ──→ [Drone flies in front]
```

## Troubleshooting

### Camera Not Detected
```bash
# List video devices
ls -la /dev/video*

# Test with v4l2
v4l2-ctl --list-devices
```

### MediaPipe Import Error
```bash
# Reinstall
pip3 uninstall mediapipe
pip3 install mediapipe>=0.10.0
```

### Gesture Not Recognized
- Check lighting (need good ambient light)
- Check camera focus (hand should be clear)
- Check distance (1-2m from camera optimal)
- Monitor `/gesture_debug` image to see detection

### Drone Not Responding
- Check WiFi connection to Tello
- Verify `/tello/status` is publishing
- Check battery level
- Test with keyboard control first: `ros2 run tello_control tello_control`

### High Latency
- Reduce camera resolution (640x480 is enough)
- Check CPU usage (should be <50%)
- Ensure USB camera on USB 3.0 port
- Close unnecessary applications

## Safety Checklist

Before Flying:
- [ ] Battery >50%
- [ ] Clear flying space (3m x 3m minimum)
- [ ] Camera has clear view
- [ ] Hand detection working (check /gesture_debug)
- [ ] Emergency gesture practiced (open palm)
- [ ] Another person present as spotter

During Flight:
- [ ] Keep hand visible to camera
- [ ] Stay within safe zone
- [ ] Monitor battery level
- [ ] Ready to use emergency gesture

## Tips for Success

1. **Start Simple**: Test hand detection without flying first
2. **Good Lighting**: Bright, even lighting works best
3. **Clear Background**: Avoid cluttered backgrounds
4. **Practice Gestures**: Get familiar with each gesture before flying
5. **Gradual Testing**: Test on ground → hovering → simple movements → full control
6. **Have Fun**: Iterate and improve based on your experience!

## Need Help?

1. Check [GESTURE_CONTROL.md](GESTURE_CONTROL.md) for implementation details
2. Review ROS2 topics: `ros2 topic list`
3. Monitor specific topics: `ros2 topic echo /gesture_command`
4. View node graph: `rqt_graph`
5. Check logs: `ros2 node info /hand_detector`

## Next Steps

Ready to implement? Start with:

1. **Implement hand_detector.py** - Get hand detection working
2. **Implement gesture_controller.py** - Map gestures to commands
3. **Test without drone** - Verify command stream
4. **Test with grounded drone** - Check physical response
5. **Fly safely!** - Start with simple movements
