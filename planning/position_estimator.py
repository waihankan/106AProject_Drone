import time
import numpy as np
import cv2
from interfaces.vision import VisionTarget
from utils.logger import setup_logger

logger = setup_logger("PositionEstimator")

class PositionEstimator:
    """
    Estimates drone position using vision-based pose estimation and velocity integration.

    Since the Tello drone doesn't provide GPS or IMU position data, this class fuses:
    1. Vision-based position from ArUco marker pose estimation (ground truth when visible)
    2. Dead reckoning from velocity command integration (continuous estimation)
    """

    def __init__(self, marker_size=0.15, vision_weight=0.7, velocity_weight=0.3):
        """
        Initialize position estimator.

        Args:
            marker_size: Physical size of ArUco markers in meters (default 0.15m = 15cm)
            vision_weight: Weight for vision-based position updates (0.0 to 1.0)
            velocity_weight: Weight for velocity integration (0.0 to 1.0)
        """
        self.marker_size = marker_size
        self.vision_weight = vision_weight
        self.velocity_weight = velocity_weight

        # Position state [x, y, z] in meters
        self.position = np.array([0.0, 0.0, 0.0])

        # Velocity state [vx, vy, vz] in m/s
        self.velocity = np.array([0.0, 0.0, 0.0])

        # Timing
        self.last_update_time = time.time()

        # Confidence metric (0.0 to 1.0)
        # High when vision recently updated, decays over time
        self.confidence = 0.0
        self.vision_update_count = 0

        logger.info(f"PositionEstimator initialized: marker_size={marker_size}m, "
                   f"vision_weight={vision_weight}, velocity_weight={velocity_weight}")

    def update_from_vision(self, target: VisionTarget, camera_matrix: np.ndarray,
                          dist_coeffs: np.ndarray) -> bool:
        """
        Update position estimate from ArUco marker pose estimation.

        Args:
            target: VisionTarget with raw_corners for pose estimation
            camera_matrix: Camera calibration matrix (3x3)
            dist_coeffs: Distortion coefficients

        Returns:
            True if position successfully updated, False otherwise
        """
        if target.raw_corners is None or len(target.raw_corners) == 0:
            logger.warning("VisionTarget missing raw_corners, cannot estimate pose")
            return False

        try:
            # Prepare marker corners for pose estimation
            # target.raw_corners is a list of 4 corner points
            corners = np.array([target.raw_corners], dtype=np.float32)

            # Estimate pose using ArUco marker
            # Returns rotation vector and translation vector (camera to marker)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                camera_matrix,
                dist_coeffs
            )

            if rvecs is None or tvecs is None:
                return False

            # Extract translation vector (camera-to-marker in camera frame)
            tvec = tvecs[0][0]  # Shape: (3,)

            # Convert to drone position relative to marker
            # In camera frame: +X=right, +Y=down, +Z=forward
            # Marker frame: Marker is at origin
            # Drone position = -tvec (invert the camera-to-marker vector)

            # Note: This gives position in camera/marker frame
            # For simplicity, we'll use this directly as our world frame
            marker_position = -tvec

            # Fuse with current position estimate
            self.position = (self.vision_weight * marker_position +
                           self.velocity_weight * self.position)

            # Update confidence (high after vision update)
            self.confidence = 1.0
            self.vision_update_count += 1

            logger.debug(f"Vision update #{self.vision_update_count}: "
                        f"position=[{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]")

            return True

        except Exception as e:
            logger.error(f"Error in vision-based pose estimation: {e}")
            return False

    def update_from_velocity(self, lr: int, fb: int, ud: int, yaw: int) -> None:
        """
        Update position estimate from velocity commands (dead reckoning).

        Args:
            lr: Left-right velocity command (-100 to 100)
            fb: Forward-backward velocity command (-100 to 100)
            ud: Up-down velocity command (-100 to 100)
            yaw: Yaw rotation velocity (-100 to 100) [not used for position]
        """
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # Convert RC commands (-100 to 100) to velocity in m/s
        # Tello max speed is approximately 8 m/s at full throttle (100)
        velocity_scale = 0.08  # 100 command units = 8 m/s

        vx = lr * velocity_scale  # Left-right velocity
        vy = fb * velocity_scale  # Forward-backward velocity
        vz = ud * velocity_scale  # Up-down velocity

        # Update velocity state
        self.velocity = np.array([vx, vy, vz])

        # Integrate velocity to update position (simple Euler integration)
        self.position += self.velocity * dt

        # Decay confidence over time (no vision update)
        # Confidence halves every 2 seconds without vision
        decay_rate = 0.3466  # Gives half-life of ~2 seconds
        self.confidence *= np.exp(-decay_rate * dt)

        logger.debug(f"Velocity update: dt={dt:.3f}s, "
                    f"vel=[{vx:.2f}, {vy:.2f}, {vz:.2f}] m/s, "
                    f"confidence={self.confidence:.2f}")

    def reset_position(self, position: np.ndarray = None) -> None:
        """
        Reset position estimate (useful when arriving at known waypoint).

        Args:
            position: New position to set, or [0,0,0] if None
        """
        if position is None:
            self.position = np.array([0.0, 0.0, 0.0])
        else:
            self.position = np.array(position, dtype=float)

        self.velocity = np.array([0.0, 0.0, 0.0])
        self.confidence = 1.0
        logger.info(f"Position reset to [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]")

    def get_position(self) -> np.ndarray:
        """
        Get current position estimate.

        Returns:
            Position as numpy array [x, y, z] in meters
        """
        return self.position.copy()

    def get_confidence(self) -> float:
        """
        Get confidence in current position estimate.

        Returns:
            Confidence value (0.0 to 1.0), where 1.0 is highest confidence
        """
        return self.confidence
