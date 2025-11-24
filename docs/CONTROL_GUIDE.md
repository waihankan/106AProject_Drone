# Hand Position Control Guide

## How It Works

Think of controlling the drone like **moving a cursor on screen** - your fist position directly controls where the drone moves.

## Visual Control Map

```
        Camera View
┌─────────────────────────┐
│         ↑ UP            │
│                         │
│  ← LEFT   [O]   RIGHT → │
│           ✊            │
│                         │
│         ↓ DOWN          │
└─────────────────────────┘

Legend:
  [O] = Center deadzone (hover)
  ✊  = Your fist position
```

## Control Behavior

### Fist Position = Drone Command

| Your Fist Position | Drone Action | Speed |
|--------------------|--------------|-------|
| **Center of frame** | Hovers in place | 0% |
| **Slightly left** | Moves left slowly | 25% |
| **Far left** | Moves left quickly | 100% |
| **Top of frame** | Ascends | Based on distance |
| **Bottom of frame** | Descends | Based on distance |
| **Top-right** | Ascends + moves right | Combined |

### Speed Control

```
Speed = Distance from center

[Center]───────────────────[Edge]
  0%        50%        100%

Near center (deadzone):
  ├─────┤  → Hover (no movement)
   80px

Beyond deadzone:
  Speed increases linearly with distance
```

## Step-by-Step Usage

### 1. Starting Up
```
1. Stand 1.5-2m from camera
2. Make sure camera can see your whole upper body
3. Show THUMBS UP (hold 2 seconds)
   → Drone takes off and hovers
```

### 2. Activating Control
```
4. Make a FIST with your dominant hand
   → Control is now ACTIVE
   → Drone will follow your fist position
```

### 3. Moving the Drone
```
5. Keep your fist closed
6. Move your ARM (not just wrist):
   - Move arm LEFT → Drone flies LEFT
   - Move arm RIGHT → Drone flies RIGHT
   - Raise arm UP → Drone ASCENDS
   - Lower arm DOWN → Drone DESCENDS

7. Return arm to center → Drone HOVERS

Tip: Start with small movements!
```

### 4. Landing
```
8. Show THUMBS DOWN
   → Drone lands immediately
```

### 5. Emergency Stop
```
Anytime: OPEN PALM (all fingers extended)
  → Drone emergency stop

OR

Anytime: Press SPACEBAR on keyboard
  → Immediate emergency stop (kills motors)
```

## Tips for Smooth Control

### 1. **Arm Movement, Not Wrist**
❌ DON'T: Just move your wrist
✅ DO: Move your whole arm for better tracking

### 2. **Smooth Movements**
❌ DON'T: Jerky, fast movements
✅ DO: Smooth, gradual arm movements

### 3. **Stay in View**
❌ DON'T: Move too far left/right out of frame
✅ DO: Stay centered in camera view

### 4. **Use the Deadzone**
❌ DON'T: Try to hold perfectly still
✅ DO: Return to center zone to hover

### 5. **Practice First**
❌ DON'T: Go full speed immediately
✅ DO: Practice with small movements

## Practice Sequence (No Drone)

Before flying, practice this sequence:

```
1. Thumbs up (hold 2s)
   → Imagine drone takes off

2. Make fist at center
   → Imagine drone hovering

3. Move fist slowly left
   → Imagine drone moving left

4. Return fist to center
   → Imagine drone stops

5. Move fist up
   → Imagine drone ascending

6. Return to center
   → Imagine drone hovering

7. Thumbs down
   → Imagine drone landing
```

## Common Mistakes & Fixes

### Mistake 1: Drone drifts when trying to hover
**Problem**: Fist not in center zone
**Fix**: Move fist to exact center of frame (green circle)

### Mistake 2: Drone moves too fast
**Problem**: Arm too far from center
**Fix**: Smaller arm movements, stay closer to center

### Mistake 3: Drone doesn't respond
**Problem**: Hand not forming proper fist
**Fix**: Make sure all fingers are curled, no fingers extended

### Mistake 4: Jerky movements
**Problem**: Moving arm too quickly
**Fix**: Slow, smooth, deliberate arm movements

### Mistake 5: Control not activating
**Problem**: Camera can't see hand clearly
**Fix**: Check lighting, adjust distance from camera

## Safety Reminders

### Before Every Flight

✅ Clear 3m x 3m flying space
✅ Battery > 50%
✅ Camera has clear view of you
✅ Test hand detection (check debug image)
✅ Practice emergency gestures
✅ Have keyboard accessible for SPACEBAR override
✅ Another person present as spotter

### During Flight

✅ Keep fist visible to camera
✅ Stay within safe zone
✅ Monitor battery level
✅ Be ready for emergency stop
✅ Start with gentle movements

### Emergency Procedures

1. **Loss of Control**
   - Open palm immediately
   - OR press SPACEBAR
   - Let drone land

2. **Camera Lost Tracking**
   - Drone will auto-hover (dead man's switch)
   - Show hand clearly to camera
   - OR manually land with thumbs down

3. **Drone Heading Toward Obstacle**
   - SPACEBAR immediately
   - OR open palm

## Visualization Screen

When running, you'll see:

```
┌──────────────────────────────────┐
│  Camera Feed with Overlays       │
│                                  │
│  ┌─────[GREEN CIRCLE]─────┐     │
│  │     (Hover Zone)        │     │
│  │                         │     │
│  │         ✊              │     │
│  │      (Your Fist)        │     │
│  └─────────────────────────┘     │
│                                  │
│  Status: FIST DETECTED           │
│  Speed: 45%                      │
│  Command: LEFT + UP              │
│  Battery: 78%                    │
└──────────────────────────────────┘

Legend:
- Green circle = Deadzone (hover)
- Hand skeleton = MediaPipe tracking
- Crosshairs = Center reference
- Status text = Current state
```

## Advanced: Adding Forward/Back Control

**Option 1: Use Hand Depth**
- Move hand toward camera = fly forward
- Move hand away from camera = fly backward
- Requires good camera depth perception

**Option 2: Two-Hand Control**
- Left fist = up/down + left/right
- Right fist = forward/back
- More complex but full 3D control

**Option 3: Tilt Detection**
- Tilt fist forward = fly forward
- Tilt fist back = fly backward
- Uses hand rotation angle

(Start with 2D control, add 3rd dimension later!)

## Expected Performance

With good conditions:
- **Latency**: 50-100ms (real-time feel)
- **Tracking**: 30 FPS (smooth)
- **Accuracy**: ±5cm positioning
- **Control Range**: Full camera view
- **Reliability**: >95% uptime

## Troubleshooting

### Hand Not Detected
1. Check lighting (need good ambient light)
2. Adjust camera angle
3. Try different hand position
4. Check MediaPipe logs

### Laggy Response
1. Close other applications
2. Use USB 3.0 port for camera
3. Reduce camera resolution (640x480 is enough)
4. Check CPU usage (<50%)

### Inaccurate Control
1. Calibrate deadzone size
2. Adjust speed multipliers
3. Add smoothing filters
4. Practice steadier hand movements

## Next Steps

Once comfortable with basic control:

1. **Add rotation control** (yaw)
2. **Add forward/backward** (3rd axis)
3. **Fine-tune speed curves** (non-linear scaling)
4. **Add gesture shortcuts** (quick commands)
5. **Multi-drone control** (track multiple fists)

## Summary

✊ **Fist = Joystick**
📹 **Camera sees you**
🎯 **Position = Command**
🚁 **Drone follows smoothly**

**Remember**: Start slow, practice without flying, be safe, and have fun!
