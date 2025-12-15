import time
import json
import numpy as np
from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple
from utils.logger import setup_logger

logger = setup_logger("TrajectoryRecorder")

@dataclass
class TrajectoryPoint:
    """Single point in the drone's trajectory"""
    timestamp: float                        # Time since mission start (seconds)
    position: List[float]                   # [x, y, z] position in meters
    target_id: Optional[int]                # Current target marker ID (None if searching)
    velocity_cmd: Tuple[int, int, int, int] # (lr, fb, ud, yaw) commands
    state: str                              # Planner state at this point
    battery: int                            # Battery percentage
    marker_visible: bool                    # Was target visible this frame?
    confidence: float                       # Position confidence (0.0 to 1.0)

class TrajectoryRecorder:
    """
    Records drone trajectory during flight for later analysis and visualization.

    Records position, commands, and state at each time step for post-flight analysis.
    """

    def __init__(self):
        """Initialize trajectory recorder"""
        self.trajectory: List[TrajectoryPoint] = []
        self.start_time: Optional[float] = None
        self.recording = False
        logger.info("TrajectoryRecorder initialized")

    def start_recording(self) -> None:
        """Start recording trajectory"""
        self.start_time = time.time()
        self.recording = True
        self.trajectory = []
        logger.info("Recording started")

    def stop_recording(self) -> None:
        """Stop recording trajectory"""
        self.recording = False
        logger.info(f"Recording stopped. Total points: {len(self.trajectory)}")

    def record_point(self, position: np.ndarray, target_id: Optional[int],
                    velocity_cmd: Tuple[int, int, int, int], state: str,
                    battery: int, marker_visible: bool, confidence: float = 1.0) -> None:
        """
        Record a single trajectory point.

        Args:
            position: Current position estimate [x, y, z] as numpy array
            target_id: ID of current target marker (None if no target)
            velocity_cmd: Velocity commands (lr, fb, ud, yaw)
            state: Current planner state as string
            battery: Battery percentage (0-100)
            marker_visible: Whether target marker is currently visible
            confidence: Position estimate confidence (0.0 to 1.0)
        """
        if not self.recording:
            return

        if self.start_time is None:
            logger.warning("Cannot record point: recording not started")
            return

        # Calculate timestamp relative to mission start
        timestamp = time.time() - self.start_time

        # Convert numpy array to list for JSON serialization
        position_list = position.tolist() if isinstance(position, np.ndarray) else list(position)

        point = TrajectoryPoint(
            timestamp=timestamp,
            position=position_list,
            target_id=target_id,
            velocity_cmd=velocity_cmd,
            state=state,
            battery=battery,
            marker_visible=marker_visible,
            confidence=confidence
        )

        self.trajectory.append(point)

        # Log every 30 points (once per second at 30Hz)
        if len(self.trajectory) % 30 == 0:
            logger.debug(f"Recorded {len(self.trajectory)} points, "
                        f"duration={timestamp:.1f}s, battery={battery}%")

    def get_trajectory(self) -> List[TrajectoryPoint]:
        """
        Get the recorded trajectory.

        Returns:
            List of TrajectoryPoint objects
        """
        return self.trajectory.copy()

    def get_duration(self) -> float:
        """
        Get total duration of recorded trajectory.

        Returns:
            Duration in seconds, or 0.0 if no points recorded
        """
        if not self.trajectory:
            return 0.0
        return self.trajectory[-1].timestamp

    def save_to_file(self, filename: str) -> bool:
        """
        Save trajectory to JSON file.

        Args:
            filename: Path to output JSON file

        Returns:
            True if successful, False otherwise
        """
        if not self.trajectory:
            logger.warning("No trajectory data to save")
            return False

        try:
            data = {
                'start_time': self.start_time,
                'duration': self.get_duration(),
                'num_points': len(self.trajectory),
                'points': [asdict(p) for p in self.trajectory]
            }

            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)

            logger.info(f"Trajectory saved to {filename} ({len(self.trajectory)} points, "
                       f"{self.get_duration():.1f}s)")
            return True

        except Exception as e:
            logger.error(f"Error saving trajectory to {filename}: {e}")
            return False

    def load_from_file(self, filename: str) -> bool:
        """
        Load trajectory from JSON file.

        Args:
            filename: Path to input JSON file

        Returns:
            True if successful, False otherwise
        """
        try:
            with open(filename, 'r') as f:
                data = json.load(f)

            self.start_time = data.get('start_time')
            self.trajectory = []

            for point_data in data.get('points', []):
                # Convert dict back to TrajectoryPoint
                # Ensure velocity_cmd is tuple
                point_data['velocity_cmd'] = tuple(point_data['velocity_cmd'])
                point = TrajectoryPoint(**point_data)
                self.trajectory.append(point)

            logger.info(f"Trajectory loaded from {filename} ({len(self.trajectory)} points)")
            return True

        except Exception as e:
            logger.error(f"Error loading trajectory from {filename}: {e}")
            return False

    def get_statistics(self) -> dict:
        """
        Get statistics about the recorded trajectory.

        Returns:
            Dictionary with trajectory statistics
        """
        if not self.trajectory:
            return {
                'num_points': 0,
                'duration': 0.0,
                'distance_traveled': 0.0,
                'avg_battery_drain': 0.0,
                'markers_visited': []
            }

        # Calculate total distance traveled
        positions = np.array([p.position for p in self.trajectory])
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        total_distance = np.sum(distances)

        # Battery drain
        battery_start = self.trajectory[0].battery
        battery_end = self.trajectory[-1].battery
        battery_drain = battery_start - battery_end

        # Unique markers visited
        markers = set(p.target_id for p in self.trajectory if p.target_id is not None)

        return {
            'num_points': len(self.trajectory),
            'duration': self.get_duration(),
            'distance_traveled': float(total_distance),
            'avg_battery_drain': float(battery_drain),
            'battery_start': battery_start,
            'battery_end': battery_end,
            'markers_visited': sorted(list(markers))
        }
