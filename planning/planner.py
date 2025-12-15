from abc import ABC, abstractmethod
from typing import List, Optional
from enum import Enum
from interfaces.vision import VisionTarget

class PlannerState(Enum):
    """States for the mission planner"""
    DISCOVERY = "discovery"           # Scanning for markers (pre-flight)
    READY = "ready"                   # Waypoints confirmed, ready to start
    APPROACHING = "approaching"       # Moving toward current waypoint
    HOVERING = "hovering"            # Holding position at waypoint
    SEARCHING = "searching"          # Lost current target, searching
    MISSION_COMPLETE = "complete"    # All waypoints visited
    ERROR = "error"                  # Unrecoverable error

class IPlanner(ABC):
    """Abstract base class for mission planners"""

    @abstractmethod
    def update(self, targets: List[VisionTarget]) -> None:
        """
        Process vision targets and update planner state.

        Args:
            targets: List of detected vision targets from current frame
        """
        pass

    @abstractmethod
    def get_current_target(self, targets: List[VisionTarget]) -> Optional[VisionTarget]:
        """
        Return the current target to follow based on planner state.

        Args:
            targets: List of detected vision targets from current frame

        Returns:
            The target to follow, or None if no target available
        """
        pass

    @abstractmethod
    def get_state(self) -> PlannerState:
        """
        Return current planner state.

        Returns:
            Current PlannerState enum value
        """
        pass

    @abstractmethod
    def start_discovery(self) -> None:
        """Start marker discovery mode (pre-flight scanning)"""
        pass

    @abstractmethod
    def confirm_waypoints(self) -> None:
        """Confirm discovered waypoints and prepare for mission start"""
        pass
