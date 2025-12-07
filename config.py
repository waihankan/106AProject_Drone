# System Configuration

# Hardware Mode: 'tello' or 'mock'
HARDWARE_MODE = 'tello'

# Vision Mode: 'aruco' or 'hand'
VISION_MODE = 'hand'

# Enable Drone Video Stream (FPV)
ENABLE_DRONE_STREAM = False

# Control Source is ALWAYS Webcam now.
# Vision runs on laptop webcam (ID 0).

# Webcam ID (default 0)
WEBCAM_ID = 0

# Tello Settings
TELLO_IP = '192.168.10.1'
TELLO_RETRY_COUNT = 3

# Safety
WATCHDOG_TIMEOUT = 2.0 # seconds
MAX_SPEED = 100

# Gesture Tuning
HAND_PINCH_RATIO = 0.4 # Distance between tips / palm size (0.0 to 1.0)
