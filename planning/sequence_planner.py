import time
from typing import List, Optional
from interfaces.vision import VisionTarget
from planning.planner import IPlanner, PlannerState
from utils.logger import setup_logger

logger = setup_logger("SequencePlanner")

class SequencePlanner(IPlanner):
    """
    Sequential waypoint planner that discovers ArUco markers in order and visits them.

    Auto-discovers markers in the order they first appear, then follows them sequentially
    with configurable hover time at each waypoint.
    """

    def __init__(self, hover_duration=3.0, position_tol=0.1, area_tol=0.01,
                 stability_frames=30, search_timeout=10.0):
        """
        Initialize sequence planner.

        Args:
            hover_duration: Seconds to hover at each waypoint (default 3.0)
            position_tol: Position tolerance for arrival detection (normalized, default 0.1)
            area_tol: Area tolerance for arrival detection (default 0.01)
            stability_frames: Frames required in position before arrival confirmed (default 30)
            search_timeout: Seconds before search times out (default 10.0)
        """
        self.hover_duration = hover_duration
        self.position_tol = position_tol
        self.area_tol = area_tol
        self.stability_frames = stability_frames
        self.search_timeout = search_timeout

        # State management
        self.state = PlannerState.DISCOVERY
        self.discovered_markers: List[int] = []  # Marker IDs in discovery order
        self.current_waypoint_index = 0
        self.desired_area = 0.05  # Default desired marker area

        # Arrival detection
        self.stable_frame_count = 0
        self.hover_start_time: Optional[float] = None

        # Search behavior
        self.search_start_time: Optional[float] = None
        self.target_lost_count = 0

        logger.info(f"SequencePlanner initialized: hover_duration={hover_duration}s, "
                   f"position_tol={position_tol}, stability_frames={stability_frames}")

    def start_discovery(self) -> None:
        """Start marker discovery mode (pre-flight scanning)"""
        self.state = PlannerState.DISCOVERY
        self.discovered_markers = []
        self.current_waypoint_index = 0
        logger.info("Discovery mode started. Show markers to camera to add waypoints.")

    def confirm_waypoints(self) -> None:
        """Confirm discovered waypoints and prepare for mission start"""
        if self.state != PlannerState.DISCOVERY:
            logger.warning("Cannot confirm waypoints: not in DISCOVERY state")
            return

        if len(self.discovered_markers) == 0:
            logger.warning("Cannot confirm waypoints: no markers discovered")
            return

        self.state = PlannerState.READY
        self.current_waypoint_index = 0
        logger.info(f"Waypoints confirmed: {len(self.discovered_markers)} markers. "
                   f"Sequence: {self.discovered_markers}")

    def update(self, targets: List[VisionTarget]) -> None:
        """
        Process vision targets and update planner state.

        Args:
            targets: List of detected vision targets from current frame
        """
        if self.state == PlannerState.DISCOVERY:
            self._update_discovery(targets)
        elif self.state == PlannerState.READY:
            # Wait for takeoff command (state will change to APPROACHING externally)
            pass
        elif self.state == PlannerState.APPROACHING:
            self._update_approaching(targets)
        elif self.state == PlannerState.HOVERING:
            self._update_hovering(targets)
        elif self.state == PlannerState.SEARCHING:
            self._update_searching(targets)
        elif self.state == PlannerState.MISSION_COMPLETE:
            pass  # Mission done
        elif self.state == PlannerState.ERROR:
            pass  # Error state, require manual intervention

    def _update_discovery(self, targets: List[VisionTarget]) -> None:
        """Discover new markers during pre-flight scanning"""
        for target in targets:
            if target.id not in self.discovered_markers:
                self.discovered_markers.append(target.id)
                waypoint_num = len(self.discovered_markers)
                logger.info(f"Waypoint #{waypoint_num} discovered: Marker ID {target.id}")

    def _update_approaching(self, targets: List[VisionTarget]) -> None:
        """Update state while approaching current waypoint"""
        current_target = self.get_current_target(targets)

        if current_target is None:
            # Lost sight of target
            self.stable_frame_count = 0
            self._transition_to_searching()
            return

        # Check if we've arrived at waypoint
        if self._is_at_waypoint(current_target):
            self.stable_frame_count += 1

            if self.stable_frame_count >= self.stability_frames:
                # Arrived! Transition to hovering
                self._transition_to_hovering()
        else:
            # Not at waypoint, reset stability counter
            self.stable_frame_count = 0

    def _update_hovering(self, targets: List[VisionTarget]) -> None:
        """Update state while hovering at waypoint"""
        current_target = self.get_current_target(targets)

        if current_target is None:
            # Lost sight of target while hovering
            self._transition_to_searching()
            return

        # Check hover timer
        if self.hover_start_time is None:
            self.hover_start_time = time.time()

        hover_elapsed = time.time() - self.hover_start_time

        if hover_elapsed >= self.hover_duration:
            # Hover complete, move to next waypoint
            self._move_to_next_waypoint()

    def _update_searching(self, targets: List[VisionTarget]) -> None:
        """Update state while searching for lost target"""
        current_target = self.get_current_target(targets)

        if current_target is not None:
            # Target reacquired!
            logger.info(f"Target reacquired: Marker ID {current_target.id}")
            self.state = PlannerState.APPROACHING
            self.search_start_time = None
            self.target_lost_count = 0
            return

        # Check search timeout
        if self.search_start_time is None:
            self.search_start_time = time.time()

        search_elapsed = time.time() - self.search_start_time

        if search_elapsed >= self.search_timeout:
            # Search timed out
            logger.error(f"Search timeout after {self.search_timeout}s. Transitioning to ERROR state.")
            self.state = PlannerState.ERROR
            self.search_start_time = None

    def _is_at_waypoint(self, target: VisionTarget) -> bool:
        """
        Check if drone is at the waypoint.

        Args:
            target: Current target marker

        Returns:
            True if at waypoint, False otherwise
        """
        # Position check (centered)
        position_ok = (abs(target.center[0]) < self.position_tol and
                      abs(target.center[1]) < self.position_tol)

        # Distance check (correct marker size/area)
        area_ok = abs(target.area - self.desired_area) < self.area_tol

        return position_ok and area_ok

    def _transition_to_hovering(self) -> None:
        """Transition to HOVERING state"""
        self.state = PlannerState.HOVERING
        self.hover_start_time = time.time()
        self.stable_frame_count = 0
        waypoint_num = self.current_waypoint_index + 1
        marker_id = self.discovered_markers[self.current_waypoint_index]
        logger.info(f"Arrived at waypoint #{waypoint_num} (Marker ID {marker_id}). "
                   f"Hovering for {self.hover_duration}s...")

    def _transition_to_searching(self) -> None:
        """Transition to SEARCHING state"""
        self.state = PlannerState.SEARCHING
        self.search_start_time = time.time()
        self.target_lost_count += 1
        marker_id = self.discovered_markers[self.current_waypoint_index]
        logger.warning(f"Lost sight of target (Marker ID {marker_id}). Searching... "
                      f"(loss #{self.target_lost_count})")

    def _move_to_next_waypoint(self) -> None:
        """Move to the next waypoint in sequence"""
        self.current_waypoint_index += 1
        self.hover_start_time = None
        self.stable_frame_count = 0

        if self.current_waypoint_index >= len(self.discovered_markers):
            # Completed all waypoints!
            self.state = PlannerState.MISSION_COMPLETE
            logger.info("Mission complete! All waypoints visited.")
        else:
            # Move to next waypoint
            self.state = PlannerState.APPROACHING
            next_marker_id = self.discovered_markers[self.current_waypoint_index]
            waypoint_num = self.current_waypoint_index + 1
            logger.info(f"Moving to waypoint #{waypoint_num} (Marker ID {next_marker_id})...")

    def get_current_target(self, targets: List[VisionTarget]) -> Optional[VisionTarget]:
        """
        Return the current target to follow based on planner state.

        Args:
            targets: List of detected vision targets from current frame

        Returns:
            The target to follow, or None if no target available
        """
        if self.state == PlannerState.DISCOVERY:
            # In discovery mode, no specific target
            return None

        if self.state == PlannerState.READY:
            # Ready but not started, no target yet
            return None

        if self.state == PlannerState.MISSION_COMPLETE or self.state == PlannerState.ERROR:
            # Mission done or error, no target
            return None

        # For APPROACHING, HOVERING, SEARCHING states, find current waypoint marker
        if self.current_waypoint_index >= len(self.discovered_markers):
            return None

        current_marker_id = self.discovered_markers[self.current_waypoint_index]

        # Find this marker in the targets list
        for target in targets:
            if target.id == current_marker_id:
                return target

        return None  # Target not visible

    def get_state(self) -> PlannerState:
        """
        Return current planner state.

        Returns:
            Current PlannerState enum value
        """
        return self.state

    def start_mission(self) -> None:
        """Start the mission (transition from READY to APPROACHING)"""
        if self.state != PlannerState.READY:
            logger.warning(f"Cannot start mission: current state is {self.state.value}")
            return

        if len(self.discovered_markers) == 0:
            logger.error("Cannot start mission: no waypoints")
            return

        self.state = PlannerState.APPROACHING
        self.current_waypoint_index = 0
        first_marker = self.discovered_markers[0]
        logger.info(f"Mission started! Approaching waypoint #1 (Marker ID {first_marker})...")

    def get_waypoint_info(self) -> dict:
        """
        Get information about current waypoint and progress.

        Returns:
            Dictionary with waypoint information
        """
        return {
            'state': self.state.value,
            'total_waypoints': len(self.discovered_markers),
            'current_waypoint': self.current_waypoint_index + 1 if self.state in [
                PlannerState.APPROACHING, PlannerState.HOVERING, PlannerState.SEARCHING
            ] else 0,
            'current_marker_id': self.discovered_markers[self.current_waypoint_index]
                                if 0 <= self.current_waypoint_index < len(self.discovered_markers)
                                else None,
            'waypoint_sequence': self.discovered_markers.copy()
        }
