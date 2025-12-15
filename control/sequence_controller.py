from interfaces.controller import IController, VisionTarget
from typing import Tuple, Optional
from control.follow_controller import FollowController
from planning.planner import IPlanner, PlannerState
from planning.position_estimator import PositionEstimator
from utils.logger import setup_logger

logger = setup_logger("SequenceController")

class SequenceController(IController):
    """
    Controller for sequential waypoint following missions.

    Integrates with SequencePlanner to follow waypoints in sequence,
    delegating low-level control to FollowController.
    """

    def __init__(self, planner: IPlanner, position_estimator: PositionEstimator, target_id=0):
        """
        Initialize sequence controller.

        Args:
            planner: Mission planner (SequencePlanner instance)
            position_estimator: Position estimator for trajectory tracking
            target_id: Default target ID (used for discovery mode)
        """
        self.planner = planner
        self.position_estimator = position_estimator
        self.target_id = target_id

        # Delegate low-level control to FollowController
        self.follow_controller = FollowController(target_id=target_id)

        logger.info("SequenceController initialized")

    def update(self, target: Optional[VisionTarget], current_state: dict,
              absolute_mode: bool = False) -> Tuple[int, int, int, int]:
        """
        Calculate drone control commands based on planner state and vision target.

        Args:
            target: The primary target to follow/track (from planner.get_current_target())
            current_state: Dictionary containing drone state (e.g., battery, height)
            absolute_mode: Force absolute mode control (default False, overridden by state)

        Returns:
            Tuple of (left_right, forward_backward, up_down, yaw) velocities (-100 to 100)
        """
        # Get current planner state
        state = self.planner.get_state()

        # State-based control logic
        if state == PlannerState.DISCOVERY:
            # Discovery mode: no movement, just hovering
            logger.debug("DISCOVERY mode: no movement")
            return (0, 0, 0, 0)

        elif state == PlannerState.READY:
            # Ready but not started: no movement
            logger.debug("READY mode: waiting for mission start")
            return (0, 0, 0, 0)

        elif state == PlannerState.SEARCHING:
            # Lost target: hover in place
            logger.debug("SEARCHING mode: hovering while searching for target")
            return (0, 0, 0, 0)

        elif state == PlannerState.HOVERING:
            # At waypoint: hold position
            # Could add small stabilization here if needed
            logger.debug("HOVERING mode: holding position at waypoint")
            return (0, 0, 0, 0)

        elif state == PlannerState.APPROACHING:
            # Approaching waypoint: use FollowController
            if target is None:
                logger.warning("APPROACHING mode but no target available")
                return (0, 0, 0, 0)

            # Delegate to FollowController with absolute mode (center + distance tracking)
            lr, fb, ud, yaw = self.follow_controller.update(target, current_state, absolute_mode=True)

            logger.debug(f"APPROACHING mode: commands=({lr}, {fb}, {ud}, {yaw})")
            return (lr, fb, ud, yaw)

        elif state == PlannerState.MISSION_COMPLETE:
            # Mission complete: no movement
            logger.debug("MISSION_COMPLETE: no movement")
            return (0, 0, 0, 0)

        elif state == PlannerState.ERROR:
            # Error state: no movement
            logger.warning("ERROR state: no movement")
            return (0, 0, 0, 0)

        else:
            # Unknown state
            logger.error(f"Unknown planner state: {state}")
            return (0, 0, 0, 0)

    def get_follow_controller(self) -> FollowController:
        """
        Get the underlying FollowController instance.

        Returns:
            FollowController instance used for low-level control
        """
        return self.follow_controller
