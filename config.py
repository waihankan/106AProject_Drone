# System Configuration
import numpy as np

# Hardware Mode: 'tello' or 'mock'
HARDWARE_MODE = 'mock'

# Vision Mode: 'aruco' or 'hand'
VISION_MODE = 'aruco'

# Enable Drone Video Stream (FPV)
ENABLE_DRONE_STREAM = False

# Control Source is ALWAYS Webcam now.
# Vision runs on laptop webcam (ID 0).

# Webcam ID (default 0)
WEBCAM_ID = 0
TARGET_ID = 2


# Tello Settings
TELLO_IP = '192.168.10.1'
TELLO_RETRY_COUNT = 3

# Safety
WATCHDOG_TIMEOUT = 2.0 # seconds
MAX_SPEED = 100

# Gesture Tuning
HAND_PINCH_RATIO = 0.25 # Distance between tips / palm size (0.0 to 1.0)

# Aruco / Camera Calibration
# Derived from user provided calibration
CAMERA_MATRIX = np.array([
    [1.43224559e+03, 0.00000000e+00, 9.60826529e+02],
    [0.00000000e+00, 1.43338093e+03, 5.35068560e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])
DISTORTION_COEFFS = np.array([
    [-9.56033057e-03, 2.86911459e-01, -3.09675371e-04, 1.00210372e-03, -6.18334570e-01]
])

# Control Parameters
DEADZONE_RATIO = 0.3
DESIRED_ARUCO_AREA = 0.05 # ~5% of screen area
 # 20% of screen dimension

# Planning Mode Configuration
ENABLE_PLANNING_MODE = False     # Enable sequential waypoint planning
ENABLE_REALTIME_VIZ = True       # Show real-time trajectory plot

# Waypoint Behavior
WAYPOINT_HOVER_DURATION = 3.0    # Seconds to hover at each waypoint
WAYPOINT_POSITION_TOL = 0.1      # Position tolerance (normalized)
WAYPOINT_AREA_TOL = 0.01         # Area tolerance (normalized)
WAYPOINT_STABILITY_FRAMES = 30   # Frames to confirm arrival (1 second at 30Hz)

# Position Estimation
ARUCO_MARKER_SIZE = 0.15         # Physical marker size in meters
VISION_FUSION_WEIGHT = 0.7       # Weight for vision-based position
VELOCITY_FUSION_WEIGHT = 0.3     # Weight for dead reckoning

# Search Behavior
SEARCH_TIMEOUT = 10.0            # Seconds before giving up search
SEARCH_PATTERN = "hover"         # "hover" or "spiral" search pattern

# Trajectory Recording & Visualization
TRAJECTORY_RECORD_RATE = 30      # Recording rate (Hz) - same as control loop
TRAJECTORY_SMOOTH_FACTOR = 2.0   # Bezier smoothing factor for post-viz
