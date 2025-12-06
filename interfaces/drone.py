from abc import ABC, abstractmethod
import numpy as np

class IDrone(ABC):
    @abstractmethod
    def connect(self):
        """Connect to the drone."""
        pass

    @abstractmethod
    def disconnect(self):
        """Disconnect and cleanup."""
        pass

    @abstractmethod
    def takeoff(self):
        """Takeoff command."""
        pass

    @abstractmethod
    def land(self):
        """Land command."""
        pass

    @abstractmethod
    def stream_on(self):
        """Start video stream."""
        pass

    @abstractmethod
    def stream_off(self):
        """Stop video stream."""
        pass

    @abstractmethod
    def get_frame(self):
        """Get the latest video frame. Returns None if no frame is available."""
        pass

    @abstractmethod
    def send_rc_control(self, left_right_velocity: int, forward_backward_velocity: int, up_down_velocity: int, yaw_velocity: int):
        """
        Send RC control commands.
        Velocities are between -100 and 100.
        """
        pass

    @abstractmethod
    def get_battery(self) -> int:
        """Get current battery percentage."""
        pass
