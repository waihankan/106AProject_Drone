# Tello Hand Gesture Control Architecture

## Overview

Control a DJI Tello drone using **hand position** detected by an external camera. The system uses MediaPipe for hand tracking and converts fist position into drone movement commands - like controlling a virtual joystick.

**Key Design**: External camera (not drone camera)
- User stands in front of camera
- Hand position = drone movement direction
- Distance from center = movement speed
- Simple, intuitive, real-time control

## System Architecture

```
┌──────────────────────┐          ┌─────────────────────────────┐
│   EXTERNAL CAMERA    │          │       TELLO DRONE           │
│   (USB Webcam)       │          │    (Flight Control)         │
└──────────┬───────────┘          └────────────┬────────────────┘
           │                                   │
           │ /camera/image_raw                 │ /tello/status
           │                                   │ /tello/battery
           │                                   │
┌──────────▼───────────────────────────────────▼────────────────┐
│                     ROS2 MIDDLEWARE                            │
└──────────┬───────────────────────────────────┬────────────────┘
           │                                   │
 ┌─────────▼──────────┐           ┌───────────▼──────────┐
 │  HAND DETECTION    │           │   GESTURE CONTROL    │
 │                    │           │                      │
 │  - MediaPipe Hands │──────────▶│  - Position Mapping  │
 │  - Gesture Recog   │           │  - Safety Checks     │
 │  - Fist Tracking   │           │  - Speed Scaling     │
 └────────────────────┘           └───────────┬──────────┘
                                              │
                                              │ /tello/control
                                              │ /tello/takeoff
                                              │ /tello/land
                                              ▼
                                   ┌─────────────────────┐
                                   │    TELLO DRIVER     │
                                   └─────────────────────┘
```

## Package Structure

```
ros_workspace/src/
├── tello/                    # Tello drone driver (existing)
├── tello_control/            # Keyboard control backup (existing)
├── tello_msg/                # Base messages (existing)
│
├── tello_interfaces/         # Custom messages
│   ├── msg/
│   │   └── HandPosition.msg       # Hand position data
│   └── package.xml
│
├── tello_vision/             # Hand detection with MediaPipe
│   ├── tello_vision/
│   │   ├── hand_detector.py       # MediaPipe hand tracking
│   │   └── gesture_recognizer.py  # Fist/thumbs detection
│   ├── config/
│   │   └── hand_params.yaml       # Detection settings
│   └── package.xml
│
├── tello_gesture_control/    # Position → velocity mapping
│   ├── tello_gesture_control/
│   │   ├── gesture_controller.py  # Main control logic
│   │   └── position_mapper.py     # Fist pos → Twist
│   ├── config/
│   │   └── control_params.yaml    # Speed, deadzone settings
│   └── package.xml
│
└── tello_safety/             # Safety monitor
    ├── tello_safety/
    │   └── safety_monitor.py      # Keyboard override, checks
    └── package.xml
```

## Data Flow

### Control Pipeline
```
Camera Image (640x480 @ 30fps)
    ↓
MediaPipe Hands Detection
    ↓
Hand Landmarks (21 keypoints)
    ↓
Gesture Recognition (fist/thumbs/palm)
    ↓
    ├─ FIST detected → Position Mapping
    │     ↓
    │  Fist Position (x, y in frame)
    │     ↓
    │  Distance from Center + Direction
    │     ↓
    │  Velocity Command (vx, vy, vz)
    │
    ├─ THUMBS UP → Takeoff command
    ├─ THUMBS DOWN → Land command
    └─ OPEN PALM → Emergency stop
    ↓
Safety Checks (battery, timeout, keyboard override)
    ↓
/tello/control (geometry_msgs/Twist)
    ↓
Tello Driver → Drone Movement
```

## Key Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/camera/image_raw` | sensor_msgs/Image | Camera feed input |
| `/hand_landmarks` | tello_interfaces/HandPosition | Raw hand position |
| `/gesture_state` | std_msgs/String | Current gesture (FIST, THUMBS_UP, etc) |
| `/fist_position` | geometry_msgs/Point | Fist position in frame |
| `/gesture_debug` | sensor_msgs/Image | Annotated camera feed |
| `/tello/control` | geometry_msgs/Twist | Velocity commands |
| `/tello/status` | tello_msg/TelloStatus | Drone status |
| `/tello/battery` | sensor_msgs/BatteryState | Battery level |
| `/emergency_stop` | std_msgs/Bool | Emergency trigger |

## Control Logic

### Position-Based Control
```
Frame divided into zones:

     [UP-LEFT]    [UP]      [UP-RIGHT]
     [LEFT]     [CENTER]     [RIGHT]
     [DOWN-LEFT]  [DOWN]    [DOWN-RIGHT]

Fist Position → Drone Movement:
  - Center (deadzone: 80px radius) → Hover
  - Left of center → Move left (speed ∝ distance)
  - Right of center → Move right
  - Top → Ascend
  - Bottom → Descend
  - Diagonal → Combined movement

Speed Scaling:
  velocity = (distance_from_center / max_distance) * max_speed
```

### Gesture Commands
| Gesture | Fingers Extended | Action |
|---------|------------------|--------|
| ✊ Fist | 0 | Position control active |
| 👍 Thumbs Up | 1 (thumb up) | Takeoff |
| 👎 Thumbs Down | 1 (thumb down) | Land |
| ✋ Open Palm | 5 | Emergency stop |

## Safety Features

### Priority Levels (Highest to Lowest)
1. **Keyboard SPACEBAR** → Immediate emergency stop
2. **Open Palm Gesture** → Controlled emergency land
3. **Dead Man's Switch** → Hover if no hand detected (2s timeout)
4. **Battery Monitor** → Auto-land at 20% battery
5. **Connection Check** → Land if WiFi signal lost
6. **Geofencing** → Max height/distance limits

### Safety Parameters
```yaml
# control_params.yaml
safety:
  deadzone_radius: 80          # pixels, center hover zone
  max_height: 2.0              # meters
  max_distance: 3.0            # meters radius
  battery_land_threshold: 20   # percent
  dead_man_timeout: 2.0        # seconds

control:
  max_speed_xy: 50             # cm/s lateral
  max_speed_z: 30              # cm/s vertical
  smoothing_alpha: 0.3         # velocity smoothing
```

## Technology Stack

### Computer Vision
- **MediaPipe Hands** (Google)
  - 21 hand landmarks per hand
  - 30+ FPS on CPU
  - Robust to lighting/backgrounds
  - Left/right hand distinction

### ROS2 Packages
- `ros-humble-cv-bridge` - Image conversion
- `ros-humble-image-transport` - Image streaming
- `ros-humble-usb-cam` - USB camera support

### Python Dependencies
```
mediapipe>=0.10.0   # Hand detection
opencv-python       # Image processing
pynput              # Keyboard monitoring
numpy<2             # Array operations
```

## Development Workflow

### Phase 1: Hand Detection
1. Set up USB camera node
2. Implement MediaPipe hand detection
3. Publish hand landmarks
4. Visualize in rqt_image_view
5. Test gesture recognition

### Phase 2: Position Control
1. Implement position mapper
2. Convert fist position → velocity
3. Add deadzone logic
4. Test without drone (publish only)
5. Visualize control zones

### Phase 3: Safety Integration
1. Implement keyboard monitor
2. Add battery checking
3. Add timeout handler
4. Test emergency stops
5. Validate all safety features

### Phase 4: Drone Testing
1. Test with drone on ground
2. Test takeoff/land gestures
3. Test hover stability (center position)
4. Test single-axis movements
5. Test combined movements
6. Tune speed parameters

## Testing Tools

### Visualization
```bash
# View camera with hand detection overlay
ros2 run rqt_image_view rqt_image_view /gesture_debug

# Monitor velocity commands
ros2 topic echo /tello/control

# View node graph
rqt_graph

# Check hand position
ros2 topic echo /fist_position
```

### Debugging
```bash
# Check if hand detected
ros2 topic hz /hand_landmarks

# Monitor gesture state
ros2 topic echo /gesture_state

# View battery level
ros2 topic echo /tello/battery

# Check safety status
ros2 topic echo /emergency_stop
```

## Performance Expectations

| Metric | Target | Notes |
|--------|--------|-------|
| Latency | <100ms | Camera → command → drone |
| FPS | 30+ | Hand detection frame rate |
| Accuracy | ±5cm | Position tracking accuracy |
| Gesture Recognition | >90% | In good lighting conditions |
| Control Response | <50ms | Command processing time |

## Common Issues & Solutions

### Hand Not Detected
- **Cause**: Poor lighting, hand too far/close
- **Fix**: Adjust lighting, stand 1.5-2m from camera

### Jerky Control
- **Cause**: No smoothing, sudden arm movements
- **Fix**: Increase smoothing_alpha, move arm slower

### Drone Drifts
- **Cause**: Fist not in center deadzone
- **Fix**: Increase deadzone_radius, practice centering

### Latency
- **Cause**: CPU overload, slow camera
- **Fix**: Close other apps, use USB 3.0, reduce resolution

## Camera Setup

### Hardware Requirements
- USB webcam (Logitech C920 or similar)
- 720p @ 30fps minimum
- Good lighting (natural or ambient)
- Stable mounting (tripod recommended)

### Camera Positioning
```
     [Camera on tripod]
           |
           | 1.5m height
           |
        [User]
         ✊

    Flying space: 3m x 3m in front of user
```

## Next Steps

1. **Implement hand_detector.py**
   - MediaPipe integration
   - Gesture classification
   - Position publishing

2. **Implement gesture_controller.py**
   - Position mapping logic
   - Safety checks
   - Velocity smoothing

3. **Implement safety_monitor.py**
   - Keyboard monitoring
   - Emergency stop logic

4. **Test & Tune**
   - Calibrate deadzone
   - Adjust speed parameters
   - Test safety features
   - Practice control

## Resources

- [MediaPipe Hands Documentation](https://google.github.io/mediapipe/solutions/hands)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Tello SDK Reference](https://github.com/dji-sdk/Tello-Python)
- [cv_bridge Tutorial](http://wiki.ros.org/cv_bridge)
