# System Configuration
import numpy as np

# Hardware Mode: 'tello' or 'mock'
HARDWARE_MODE = 'tello'

# Vision Mode: 'aruco' or 'hand'
VISION_MODE = 'aruco'

# Enable Drone Video Stream (FPV)
ENABLE_DRONE_STREAM = True

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


DRONE_CAMERA_MATRIX = np.array([
    [580.0,   0.0, 480.0],
    [  0.0, 580.0, 360.0],
    [  0.0,   0.0,   1.0]
])
DRONE_DISTORTION_COEFFS = np.zeros((5, 1))
DRONE_MARKER_SIZE = 0.13


# Control Parameters
DEADZONE_RATIO = 0.3
DESIRED_ARUCO_AREA = 0.05 # ~5% of screen area
 # 20% of screen dimension
