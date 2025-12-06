from abc import ABC, abstractmethod
from typing import Tuple, List, Optional
from .vision import VisionTarget

class IController(ABC):
    @abstractmethod
    def update(self, target: Optional[VisionTarget], current_state: dict) -> Tuple[int, int, int, int]:
        """
        Calculate drone control commands based on vision target and current state.
        
        Args:
            target: The primary target to follow/track (can be None).
            current_state: Dictionary containing drone state (e.g., battery, height).
            
        Returns:
            Tuple of (left_right, forward_backward, up_down, yaw) velocities (-100 to 100).
        """
        pass
