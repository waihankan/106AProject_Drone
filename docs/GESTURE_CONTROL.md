# Hand Gesture Drone Control

## Goal
Control the Tello drone using hand gestures detected by an external camera. Simple, reliable, and robust.

## Architecture

```
┌─────────────────────┐
│  EXTERNAL CAMERA    │  (USB webcam / laptop camera)
│  (pointing at user) │
└──────────┬──────────┘
           │
           │ /camera/image_raw
           │
┌──────────▼────────────────┐
│   HAND DETECTION NODE     │  (MediaPipe Hands)
│   - Detect hand landmarks │
│   - Recognize gestures    │
└──────────┬────────────────┘
           │
           │ /gesture_command (String: "forward", "back", "up", "down", "left", "right", "land", "takeoff")
           │
┌──────────▼────────────────┐
│  GESTURE CONTROL NODE     │
│   - Map gestures → speed  │
│   - Safety checks         │
│   - Smooth transitions    │
└──────────┬────────────────┘
           │
           │ /tello/control (Twist)
           │ /tello/takeoff, /tello/land
           │
┌──────────▼────────────────┐
│     TELLO DRIVER          │
└───────────────────────────┘
```

## Package Structure

```
ros_workspace/src/
├── tello/                      # Existing tello driver
├── tello_msg/                  # Existing messages
│
├── tello_interfaces/           # NEW: Custom messages
│   ├── msg/
│   │   ├── GestureCommand.msg      # gesture name + confidence
│   │   └── HandLandmarks.msg       # hand pose data (optional)
│   └── package.xml
│
├── tello_vision/               # NEW: Hand detection
│   ├── tello_vision/
│   │   ├── hand_detector.py        # MediaPipe detection node
│   │   └── gesture_recognizer.py   # Classify hand poses
│   ├── config/
│   │   └── gestures.yaml           # Gesture definitions
│   └── package.xml
│
└── tello_gesture_control/      # NEW: Gesture → drone commands
    ├── tello_gesture_control/
    │   ├── gesture_controller.py   # Main control logic
    │   └── safety_monitor.py       # Battery/signal checks
    ├── config/
    │   └── control_params.yaml     # Speed settings, etc.
    └── package.xml
```

## Gesture Mapping (Position-Based Control)

### Core Concept: "Control the Drone Like a Joystick"
- **✊ Fist = Control Active** (like holding a joystick)
- **Arm Position** = Drone movement direction & speed
- **No Fist** = Drone stops/hovers (safety feature)

### Control Modes

#### Mode 1: Fist Position Control (Primary)
```
Camera View (Divide into 3x3 grid):

     [Up-Left]  [Up]     [Up-Right]
     [Left]     [CENTER] [Right]
     [Down-Left][Down]   [Down-Right]

Fist Position → Drone Movement:
- Center zone → Hover in place
- Left zone → Fly left (speed based on distance from center)
- Right zone → Fly right
- Top zone → Ascend
- Bottom zone → Descend
- Top-Left → Ascend + Left
- Etc. (all 8 directions + hover)
```

**Speed Control**: Distance from center = speed
- Near center (deadzone) → slow/hover
- Far from center → faster movement
- Max speed at edge of frame

#### Mode 2: State Gestures (for commands)
| Gesture | Command | Description |
|---------|---------|-------------|
| 👍 Thumbs Up | TAKEOFF | Start flying (hold 2s) |
| 👎 Thumbs Down | LAND | Land immediately |
| ✋ Open Palm | EMERGENCY | Emergency stop |
| ✊ Fist | ACTIVE CONTROL | Enable position control |

### Example Control Flow
```python
1. Show thumbs up → Drone takes off
2. Make fist → Control activated
3. Move fist left → Drone moves left
4. Move fist up-right → Drone ascends and moves right
5. Return fist to center → Drone hovers
6. Release fist (open hand) → Drone hovers (safety)
7. Thumbs down → Drone lands
```

## Data Flow

### 1. Detection Pipeline
```python
Camera Image (640x480 @ 30fps)
    ↓
MediaPipe Hands Detection
    ↓
Hand Landmarks (21 keypoints per hand)
    ↓
Gesture Recognition (rule-based or ML)
    ↓
/gesture_command (String)
```

### 2. Control Pipeline
```python
/gesture_command + /tello/status
    ↓
Safety Checks (battery, connection)
    ↓
Command Smoothing (avoid jitter)
    ↓
Speed Mapping (gesture → velocity)
    ↓
/tello/control (Twist message)
```

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | Camera feed |
| `/hand_landmarks` | tello_interfaces/HandLandmarks | Raw hand pose |
| `/gesture_command` | std_msgs/String | Recognized gesture |
| `/gesture_debug` | sensor_msgs/Image | Annotated image |
| `/tello/control` | geometry_msgs/Twist | Velocity commands |
| `/tello/status` | tello_msg/TelloStatus | Drone state |

## Safety Features

### Essential Safety
1. **Keyboard Override**: Press SPACEBAR for immediate emergency stop (highest priority)
2. **Dead Man's Switch**: Must show hand to keep flying (if no hand detected for 2s → hover)
3. **Battery Monitor**: Auto-land at 20% battery
4. **Connection Check**: Land if WiFi drops
5. **Height Limit**: Max 2m altitude
6. **Geofence**: Stay within 3m radius
7. **Emergency Gesture**: Instant land with open palm

### Safety Priority (highest to lowest)
1. **Keyboard SPACEBAR** → Immediate emergency stop (kills motors)
2. **Emergency gesture** (open palm) → Controlled land
3. **Dead man's switch** → Hover in place
4. **Battery/connection** → Auto-land

### Gesture Filtering
- **Confidence Threshold**: Only act on gestures >80% confidence
- **Temporal Smoothing**: Require gesture stable for 0.5s
- **Hysteresis**: Avoid rapid switching between gestures

## Technology Stack

### Computer Vision
- **MediaPipe Hands** (Google) - Production-ready, fast, accurate
- Runs at 30+ FPS on CPU
- 21 hand landmarks per hand
- Left/right hand distinction

### ROS2 Packages Needed
```bash
# Python dependencies
pip install mediapipe opencv-python numpy

# ROS2 packages
ros-humble-cv-bridge
ros-humble-image-transport
ros-humble-usb-cam  # for USB camera
```

## Development Workflow

### Phase 1: Hand Detection (Week 1)
1. Set up USB camera with ROS2
2. Create `hand_detector.py` with MediaPipe
3. Publish hand landmarks
4. Visualize detection in rqt_image_view
5. Test different lighting conditions

### Phase 2: Gesture Recognition (Week 1)
1. Define 6-8 core gestures
2. Implement gesture recognizer (rule-based)
3. Test gesture reliability
4. Add confidence thresholds
5. Tune for robustness

### Phase 3: Control Integration (Week 2)
1. Create `gesture_controller.py`
2. Map gestures to Twist commands
3. Add command smoothing
4. Test without drone (publish only)
5. Verify command stream stability

### Phase 4: Drone Testing (Week 2)
1. Start with drone on ground
2. Test takeoff/land gestures
3. Test hover stability
4. Test movement commands (one at a time)
5. Integrate full control loop

### Phase 5: Safety & Tuning (Week 3)
1. Add dead man's switch
2. Implement emergency stop
3. Tune speed parameters
4. Test edge cases
5. User testing & feedback

## Example Position-Based Control Implementation

```python
import numpy as np

class PositionController:
    def __init__(self, frame_width=640, frame_height=480):
        self.frame_width = frame_width
        self.frame_height = frame_height

        # Define center zone (deadzone for hovering)
        self.center_x = frame_width / 2
        self.center_y = frame_height / 2
        self.deadzone_radius = 80  # pixels

        # Max speeds (in cm/s or appropriate units)
        self.max_speed_xy = 50  # lateral movement
        self.max_speed_z = 30   # vertical movement

    def recognize_gesture(self, hand_landmarks):
        """Recognize hand gesture type"""
        extended_fingers = self.count_extended_fingers(hand_landmarks)

        thumb_tip = hand_landmarks[4]
        wrist = hand_landmarks[0]

        # Thumbs up: thumb extended upward, others closed
        if extended_fingers == 1 and thumb_tip.y < wrist.y:
            return "TAKEOFF"

        # Thumbs down: thumb extended downward, others closed
        if extended_fingers == 1 and thumb_tip.y > wrist.y:
            return "LAND"

        # Open palm: all fingers extended
        if extended_fingers >= 5:
            return "EMERGENCY"

        # Fist: no fingers extended
        if extended_fingers == 0:
            return "FIST"  # Active control mode

        return "IDLE"

    def fist_to_velocity(self, hand_landmarks, frame_width, frame_height):
        """Convert fist position to velocity commands"""

        # Get wrist position (more stable than fingertips for fist)
        wrist = hand_landmarks[0]

        # Convert normalized coordinates to pixel coordinates
        fist_x = wrist.x * frame_width
        fist_y = wrist.y * frame_height

        # Calculate distance from center
        dx = fist_x - self.center_x
        dy = fist_y - self.center_y
        distance = np.sqrt(dx**2 + dy**2)

        # Check if in deadzone
        if distance < self.deadzone_radius:
            return {
                'vx': 0.0,  # left/right
                'vy': 0.0,  # forward/back
                'vz': 0.0,  # up/down
                'mode': 'HOVER'
            }

        # Calculate normalized direction and speed
        # Horizontal movement (left/right)
        vx = (dx / (frame_width/2)) * self.max_speed_xy
        vx = np.clip(vx, -self.max_speed_xy, self.max_speed_xy)

        # Vertical movement (up/down) - note: y increases downward in image
        vz = -(dy / (frame_height/2)) * self.max_speed_z
        vz = np.clip(vz, -self.max_speed_z, self.max_speed_z)

        # Optional: Use hand depth (z coordinate) for forward/back
        # or add another gesture for forward/back control
        vy = 0.0  # Forward/back (can be added later)

        return {
            'vx': vx,
            'vy': vy,
            'vz': vz,
            'mode': 'POSITION_CONTROL',
            'fist_pos': (fist_x, fist_y),
            'distance_from_center': distance
        }

    def count_extended_fingers(self, hand_landmarks):
        """Count how many fingers are extended"""
        # Finger tip landmarks: thumb=4, index=8, middle=12, ring=16, pinky=20
        # Finger base landmarks: thumb=2, index=6, middle=10, ring=14, pinky=18

        finger_tips = [4, 8, 12, 16, 20]
        finger_bases = [2, 6, 10, 14, 18]

        extended = 0
        for tip, base in zip(finger_tips, finger_bases):
            # Finger is extended if tip is higher than base (smaller y value)
            if hand_landmarks[tip].y < hand_landmarks[base].y - 0.05:
                extended += 1

        return extended

# Usage in main control loop:
def control_loop():
    controller = PositionController()

    while True:
        # Get hand landmarks from MediaPipe
        hand_landmarks = detect_hand()

        if hand_landmarks is None:
            # No hand detected → Safety: hover or land
            publish_velocity(0, 0, 0)
            continue

        # Recognize gesture type
        gesture = controller.recognize_gesture(hand_landmarks)

        if gesture == "TAKEOFF":
            publish_takeoff()
        elif gesture == "LAND":
            publish_land()
        elif gesture == "EMERGENCY":
            publish_emergency_stop()
        elif gesture == "FIST":
            # Active control: convert fist position to velocity
            velocity = controller.fist_to_velocity(
                hand_landmarks,
                frame_width=640,
                frame_height=480
            )
            publish_velocity(velocity['vx'], velocity['vy'], velocity['vz'])
        else:
            # Not fist → hover
            publish_velocity(0, 0, 0)
```

### Visualization Helper
```python
def draw_control_zones(image, controller):
    """Draw visual feedback on image"""
    import cv2

    # Draw center deadzone circle
    cv2.circle(
        image,
        (int(controller.center_x), int(controller.center_y)),
        controller.deadzone_radius,
        (0, 255, 0),  # Green
        2
    )

    # Draw center crosshair
    cv2.line(image,
             (int(controller.center_x - 20), int(controller.center_y)),
             (int(controller.center_x + 20), int(controller.center_y)),
             (0, 255, 0), 2)
    cv2.line(image,
             (int(controller.center_x), int(controller.center_y - 20)),
             (int(controller.center_x), int(controller.center_y + 20)),
             (0, 255, 0), 2)

    # Draw quadrant lines
    cv2.line(image, (0, int(controller.center_y)),
             (controller.frame_width, int(controller.center_y)),
             (100, 100, 100), 1)
    cv2.line(image, (int(controller.center_x), 0),
             (int(controller.center_x), controller.frame_height),
             (100, 100, 100), 1)

    # Add text labels
    cv2.putText(image, "UP", (int(controller.center_x - 15), 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(image, "DOWN", (int(controller.center_x - 25),
                controller.frame_height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(image, "LEFT", (10, int(controller.center_y)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(image, "RIGHT", (controller.frame_width - 70,
                int(controller.center_y)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return image
```

## Testing Without Drone

```bash
# Terminal 1: Start camera
ros2 run usb_cam usb_cam_node

# Terminal 2: Start hand detection
ros2 run tello_vision hand_detector

# Terminal 3: Visualize
ros2 run rqt_image_view rqt_image_view /gesture_debug

# Terminal 4: Monitor commands
ros2 topic echo /gesture_command

# Terminal 5: Test controller (without drone)
ros2 run tello_gesture_control gesture_controller --ros-args -p test_mode:=true
```

## Camera Setup Recommendations

### Hardware
- **Webcam**: Logitech C920/C922 or similar (1080p @ 30fps)
- **Mounting**: Stable tripod at eye level
- **Distance**: 1-2 meters from user
- **Lighting**: Good ambient light, avoid backlighting

### Camera Position
```
     [Camera]
        |
        | 1.5m
        |
     [User] -----> [Drone flies in front]
```

## Next Steps

1. **Install dependencies**
   ```bash
   pip install mediapipe opencv-python
   ```

2. **Set up USB camera node**
   ```bash
   sudo apt install ros-humble-usb-cam
   ```

3. **Create packages**
   ```bash
   cd ~/ros_workspace/src
   ros2 pkg create --build-type ament_python tello_vision
   ros2 pkg create --build-type ament_python tello_gesture_control
   ```

4. **Start with hand detection**
   - Get camera feed in ROS2
   - Implement MediaPipe detection
   - Visualize hand landmarks
   - Test gesture recognition

5. **Build control layer**
   - Map gestures to commands
   - Add safety features
   - Test without drone

6. **Integrate with Tello**
   - Test with drone on ground
   - Gradual flight testing
   - Tune and improve

## Expected Performance

- **Latency**: <100ms (camera → gesture → command)
- **FPS**: 30+ frames/second
- **Gesture Recognition**: >90% accuracy in good conditions
- **Control Responsiveness**: Smooth, no lag
- **Safety**: Multiple failsafes active

## Challenges & Solutions

| Challenge | Solution |
|-----------|----------|
| Poor lighting | Add brightness adjustment, recommend good lighting |
| Hand occlusion | Use temporal smoothing, maintain last known gesture |
| False positives | Require gesture stability (0.5s), high confidence threshold |
| Latency | Optimize pipeline, use efficient hardware |
| Multiple hands | Track dominant hand only, or allow hand switching |
| Fatigue | Design ergonomic gestures, support multiple control modes |
