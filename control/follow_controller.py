from interfaces.controller import IController, VisionTarget
from typing import Tuple, Optional
from utils.logger import setup_logger

logger = setup_logger("FollowController")

class FollowController(IController):
    def __init__(self, target_id=0):
        self.target_id = target_id
        
        # PID Constants (Simple P for now)
        self.k_yaw = 40.0       # Rotation speed gain
        self.k_throttle = 50.0  # Vertical speed gain
        self.k_pitch = 50.0     # Forward/Backward speed gain
        
        self.desired_area = 0.05 # roughly 5% of screen area
        self.dead_zone = 0.1 # 10% center deadzone to prevent jitter

        # Safety & Smoothing State
        self.prev_yaw = 0
        self.prev_ud = 0
        self.prev_fb = 0
        
        # Smoothing factors (0.0 - 1.0)
        # Lower = smoother but more lag. Higher = more responsive but jittery.
        self.alpha_target = 0.7 
        self.alpha_vel = 0.5    
        
        # Slew Rate Limits (max change per tick)
        # At 30Hz, 20 means max change of 600 velocity units per second (very fast)
        # Lower this to make it "heavier"
        self.max_accel = 30 

        # Smoothed Target State
        self.smooth_x = 0.0
        self.smooth_y = 0.0
        self.smooth_area = 0.0
        self.has_locked = False

    def _apply_ema(self, current, new, alpha):
        return (alpha * new) + ((1 - alpha) * current)

    def _apply_slew_limit(self, current_val, desired_val, max_change):
        diff = desired_val - current_val
        limited_diff = max(-max_change, min(max_change, diff))
        return int(current_val + limited_diff)

    def update(self, target: Optional[VisionTarget], current_state: dict) -> Tuple[int, int, int, int]:
        """
        Returns (lr, fb, ud, yaw)
        """
        if target is None or target.id != self.target_id:
            # If we lose target, we should probably decay velocity to 0 smoothly
            # rather than hard cutting, but for safety, hard stop 0 is okay for "lost"
            # resets limits to 0 immediately for safety
            self.prev_yaw = 0
            self.prev_ud = 0
            self.prev_fb = 0
            self.has_locked = False
            return 0, 0, 0, 0

        # 1. Smooth the Input (Target Noise)
        if not self.has_locked:
            self.smooth_x = target.center[0]
            self.smooth_y = target.center[1]
            self.smooth_area = target.area
            self.has_locked = True
        else:
            self.smooth_x = self._apply_ema(self.smooth_x, target.center[0], self.alpha_target)
            self.smooth_y = self._apply_ema(self.smooth_y, target.center[1], self.alpha_target)
            self.smooth_area = self._apply_ema(self.smooth_area, target.area, self.alpha_target)

        # 2. Calculate Desired Velocity (P-Control)
        desired_yaw = 0
        if abs(self.smooth_x) > self.dead_zone:
            desired_yaw = int(self.smooth_x * self.k_yaw)

        # Tello: Positive up_down is UP. 
        # If target is at top (-1), we need to go UP (+). So -target.y
        desired_ud = 0
        if abs(self.smooth_y) > self.dead_zone:
            desired_ud = int(-self.smooth_y * self.k_throttle)

        # PID for Distance
        error_dist = self.desired_area - self.smooth_area
        desired_fb = 0
        # Only move if error is significant
        if abs(error_dist) > 0.01:
            desired_fb = int(error_dist * 100 * self.k_pitch) 

        # 3. Apply Slew Rate Limiting (Output Smoothing)
        # Prevents "jerky" reaction to sudden target jumps
        final_yaw = self._apply_slew_limit(self.prev_yaw, desired_yaw, self.max_accel)
        final_ud = self._apply_slew_limit(self.prev_ud, desired_ud, self.max_accel)
        final_fb = self._apply_slew_limit(self.prev_fb, desired_fb, self.max_accel)

        # 4. Clamp final values
        final_yaw = max(-100, min(100, final_yaw))
        final_ud = max(-100, min(100, final_ud))
        final_fb = max(-100, min(100, final_fb))

        # Store for next tick
        self.prev_yaw = final_yaw
        self.prev_ud = final_ud
        self.prev_fb = final_fb

        return 0, final_fb, final_ud, final_yaw
