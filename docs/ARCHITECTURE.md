# Tello Computer Vision Control Architecture

## Overview

This architecture uses an **EXTERNAL CAMERA** for computer vision (not the drone's onboard camera). The external camera tracks the drone and/or environment, providing positioning and control commands to the Tello.

**Key Design Decision**: External camera allows for:
- Better field of view (track drone + environment)
- More powerful CV processing (no bandwidth limits)
- Easier development/debugging
- Multiple drone control from single vision system

## System Architecture

```
┌──────────────────────┐          ┌─────────────────────────────┐
│   EXTERNAL CAMERA    │          │       TELLO DRONE           │
│   (USB/Network)      │          │    (Flight Control Only)    │
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
 │  VISION PIPELINE   │           │   CONTROL LOGIC      │
 │                    │           │                      │
 │  - Drone Detection │──────────▶│  - Position Estimator│
 │  - Object Detection│           │  - State Machine     │
 │  - Pose Estimation │           │  - Path Planner      │
 │  - Scene Tracking  │           │  - PID Controllers   │
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

## Recommended Package Structure

```
ros_workspace/src/
├── tello/                    # Base tello driver (existing)
├── tello_control/            # Keyboard control (existing)
├── tello_msg/                # Custom messages (existing)
│
├── tello_vision/             # NEW: Vision processing package
│   ├── tello_vision/
│   │   ├── yolo_detector.py       # YOLO object detection
│   │   ├── pose_estimator.py      # MediaPipe/pose estimation
│   │   ├── tracker.py              # Object tracking
│   │   └── vision_node.py          # Main vision node
│   ├── config/
│   │   ├── yolo_config.yaml
│   │   └── detection_params.yaml
│   └── package.xml
│
├── tello_autonomy/           # NEW: Autonomous control package
│   ├── tello_autonomy/
│   │   ├── mission_planner.py     # High-level mission logic
│   │   ├── state_machine.py       # Drone state management
│   │   ├── controllers.py         # PID/control algorithms
│   │   └── autonomy_node.py       # Main autonomy node
│   ├── config/
│   │   └── controller_params.yaml
│   └── package.xml
│
├── tello_behaviors/          # NEW: Pre-built behaviors
│   ├── tello_behaviors/
│   │   ├── follow_person.py       # Person following behavior
│   │   ├── gesture_control.py     # Hand gesture control
│   │   ├── object_tracking.py     # Track specific objects
│   │   └── landing_assistance.py  # Vision-assisted landing
│   └── package.xml
│
└── tello_interfaces/         # NEW: Custom messages/services
    ├── msg/
    │   ├── Detection.msg           # Single detection
    │   ├── DetectionArray.msg      # Multiple detections
    │   ├── TrackedObject.msg       # Tracked object info
    │   └── DroneCommand.msg        # High-level commands
    ├── srv/
    │   ├── StartMission.srv
    │   └── SetTarget.srv
    └── package.xml
```

## Data Flow

### 1. Vision Pipeline
```
/image_raw (sensor_msgs/Image)
    ↓
tello_vision/yolo_detector
    ↓
/detections (DetectionArray)
    ↓
tello_vision/tracker
    ↓
/tracked_objects (TrackedObject[])
```

### 2. Control Pipeline
```
/tracked_objects + /status + /battery
    ↓
tello_autonomy/state_machine
    ↓
/drone_command (high-level)
    ↓
tello_autonomy/controllers
    ↓
/control (geometry_msgs/Twist)
```

## Key Topics & Messages

### Vision Topics
- `/detections` - Detected objects with bounding boxes
- `/tracked_objects` - Objects tracked across frames
- `/target_position` - Primary target position/velocity
- `/debug_image` - Annotated image for visualization

### Control Topics
- `/mission_status` - Current mission state
- `/drone_command` - High-level commands (follow, track, land)
- `/control` - Low-level velocity commands (existing)

### Monitoring Topics
- `/vision/fps` - Vision processing FPS
- `/autonomy/state` - Current autonomy state

## Technology Choices

### For Object Detection
1. **YOLOv8/YOLOv11** (Recommended)
   - Fast, accurate, good for drones
   - Use `ultralytics` Python package
   - Pre-trained on COCO dataset

2. **MediaPipe** (For human detection/pose)
   - Excellent for person tracking
   - Real-time performance
   - Built-in pose estimation

3. **Google's ViT/PaliGemma** (Advanced)
   - For more complex understanding
   - Requires more compute power

### For Tracking
- **SORT/DeepSORT** - Multi-object tracking
- **ByteTrack** - Modern, fast tracker
- Custom Kalman filter for simple cases

## Safety Features

Essential safety mechanisms:

1. **Battery Monitor** - Auto-land at low battery
2. **Connection Monitor** - Land if signal lost
3. **Geofencing** - Stay within safe zone
4. **Emergency Stop** - Kill switch via topic
5. **Timeout Handler** - Land if no commands received
6. **Collision Avoidance** - Stop if obstacle detected

## Getting Started Workflow

### Phase 1: Vision Setup
1. Create `tello_vision` package
2. Implement YOLO detector node
3. Test with recorded bag files
4. Visualize detections in RViz/rqt

### Phase 2: Basic Control
1. Create `tello_autonomy` package
2. Implement simple state machine
3. Create basic "center object" controller
4. Test in simulation or safe environment

### Phase 3: Behaviors
1. Create `tello_behaviors` package
2. Implement person following
3. Add gesture recognition
4. Test complete pipeline

### Phase 4: Advanced Features
1. Add multi-object tracking
2. Implement path planning
3. Add obstacle avoidance
4. Tune PID controllers

## Example Use Cases

### 1. Person Following
```python
# User starts mission
/mission_status: "following_person"
# Vision detects person → /detections
# Tracker maintains ID → /tracked_objects
# Controller keeps person centered → /control
```

### 2. Gesture Control
```python
# Detect hand gestures via MediaPipe
# Map gestures to commands:
#   - Thumbs up → takeoff
#   - Wave → follow me
#   - Stop sign → land
```

### 3. Object Tracking
```python
# User selects object in image
# Tracker locks onto object
# Drone maintains distance and angle
```

## Development Tools

### Visualization
- `rqt_image_view` - View camera feed
- `rqt_graph` - Visualize node connections
- `rviz2` - 3D visualization
- `plotjuggler` - Real-time plotting

### Testing
- `ros2 bag` - Record/replay data
- `pytest` - Unit testing
- Simulation tools (Gazebo if needed)

### Debugging
- `ros2 topic echo` - Monitor messages
- `ros2 node info` - Node details
- `ros2 param` - Runtime parameters

## Next Steps

1. **Choose your CV model** (YOLO recommended for start)
2. **Create custom message types** (Detection, TrackedObject)
3. **Build vision package** with detector + tracker
4. **Test vision offline** with recorded images
5. **Add simple controller** (center object)
6. **Integrate with drone** and test in safe space
7. **Iterate and improve**

## Resources

- YOLOv8: https://docs.ultralytics.com/
- MediaPipe: https://google.github.io/mediapipe/
- ROS2 Vision: https://github.com/ros-perception/vision_opencv
- cv_bridge: For ROS↔OpenCV conversion
