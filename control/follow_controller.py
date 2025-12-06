from interfaces.controller import IController, VisionTarget
from typing import Tuple, Optional
from utils.logger import setup_logger

logger = setup_logger("FollowController")

class FollowController(IController):
    def __init__(self, target_id=0):
        self.target_id = target_id
        
        # PID Constants (Simple P for now)
        # Unit Scaling Explanation:
        # LR/UD inputs are Normalized Screen Coordinates (-1.0 to 1.0). Typical Action Delta: 0.5.
        # FB Input is Linear Palm Size (0.0 to 0.5). Typical Action Delta: 0.02.
        # To get similar Output Velocity (e.g. 50), we need different gains.
        # LR: 0.5 * 100 = 50. Gain ~100-150.
        # FB: 0.02 * 2500 = 50. Gain ~2500-3000.
        
        self.k_yaw = 150.0       # Rotation speed gain
        self.k_throttle = 150.0  # Vertical speed gain
        self.k_pitch = 50.0      # Forward/Backward speed gain (Base gain, multiplied by 60 later -> 3000 effective)

        self.desired_area = 0.05 
        self.dead_zone = 0.1     # 10% center deadzone

        # Safety & Smoothing State
        self.prev_yaw = 0
        self.prev_ud = 0
        self.prev_fb = 0
        
        # Smoothing factors (0.0 - 1.0)
        # Lower = smoother but more lag. Higher = more responsive but jittery.
        self.alpha_target = 0.15 # Low alpha = Heavy Smoothing
        self.alpha_vel = 0.5    
        
        # Slew Rate Limits (max change per tick)
        # At 30Hz, 20 means max change of 600 velocity units per second (very fast)
        # Lower this to make it "heavier"
        self.max_accel = 10 # Reduced to make reaction slower/smoother 

        # Smoothed Target State
        self.smooth_x = 0.0
        self.smooth_y = 0.0
        self.smooth_area = 0.0
        
        # Reference State (Set on Lock)
        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_area = 0.0
        
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
            # Resets
            self.prev_yaw = 0
            self.prev_ud = 0
            self.prev_fb = 0
            self.has_locked = False
            return 0, 0, 0, 0

        # 1. Handle Reference Point (Lock Logic)
        if not self.has_locked:
            # First frame of pinch: Set Reference
            self.ref_x = target.center[0]
            self.ref_y = target.center[1]
            self.ref_area = target.area
            
            # Initialize smoothers to current to avoid jump
            self.smooth_x = target.center[0]
            self.smooth_y = target.center[1]
            self.smooth_area = target.area
            
            self.has_locked = True
        else:
            # Apply Smoothers
            self.smooth_x = self._apply_ema(self.smooth_x, target.center[0], self.alpha_target)
            self.smooth_y = self._apply_ema(self.smooth_y, target.center[1], self.alpha_target)
            self.smooth_area = self._apply_ema(self.smooth_area, target.area, self.alpha_target)

        # 2. Calculate Desired Velocity (Relative to Reference Point)
        
        # Calculate Deltas
        delta_x = self.smooth_x - self.ref_x
        delta_y = self.smooth_y - self.ref_y
        delta_area = self.smooth_area - self.ref_area

        # STRAFE Control (Left/Right)
        # If I move Right (positive x), delta_x is positive. I want to strafe Right.
        desired_lr = 0
        if abs(delta_x) > self.dead_zone:
             desired_lr = int(delta_x * self.k_yaw) 

        # Height control (Up/Down)
        # Tello: Positive up_down is UP. 
        # Video: Y is Top(-1) to Bottom(1).
        # Move Hand UP (Negative Y change) -> delta_y is Negative.
        # We want Drone UP (Positive). So invert delta.
        desired_ud = 0
        if abs(delta_y) > self.dead_zone:
            desired_ud = int(-delta_y * self.k_throttle)

        # PID for Distance (Forward/Backward)
        # Requested: Inverted logic.
        # Original: error = desired - current. 
        # New Logic: Compare to Reference Area.
        # If Hand gets CLOSE (Area Increases): delta_area is Positive.
        # User wants "Opposite" of previous behavior in context of "Drone facing other way".
        # PID for Distance (Forward/Backward)
        # Using Robust Palm Scale (Linear). 
        # Hand Bigger (Closer) -> delta_area is POSITIVE.
        desired_fb = 0
        if abs(delta_area) > 0.002:
            # Linear values are small (~0.02 change). 
            # Previous gains: 500->60->30.
            # User requested even less sensitivity.
            # New gain: 20 * 50 = 1000.
            desired_fb = int(delta_area * 20 * self.k_pitch) 
            
        # WINNER TAKES ALL Logic
        # Decouple inputs: Only allow the strongest axis to control the drone.
        abs_lr = abs(desired_lr)
        abs_fb = abs(desired_fb)
        abs_ud = abs(desired_ud)
        
        # Determine dominant axis
        max_val = max(abs_lr, abs_fb, abs_ud)
        
        # Threshold to avoid noise switching
        if max_val > 10: 
            if max_val == abs_lr:
                desired_fb = 0
                desired_ud = 0
            elif max_val == abs_fb:
                desired_lr = 0
                desired_ud = 0
            elif max_val == abs_ud:
                desired_lr = 0
                desired_fb = 0
        else:
            # If nothing is strong enough, stop all
            desired_lr = 0
            desired_fb = 0
            desired_ud = 0 

        # 3. Apply Slew Rate Limiting (Output Smoothing)
        final_lr = self._apply_slew_limit(self.prev_yaw, desired_lr, self.max_accel)
        final_ud = self._apply_slew_limit(self.prev_ud, desired_ud, self.max_accel)
        final_fb = self._apply_slew_limit(self.prev_fb, desired_fb, self.max_accel)

        # 4. Clamp final values
        final_lr = max(-100, min(100, final_lr))
        final_ud = max(-100, min(100, final_ud))
        final_fb = max(-100, min(100, final_fb))

        # Store for next tick
        self.prev_yaw = final_lr
        self.prev_ud = final_ud
        self.prev_fb = final_fb

        # Return (lr, fb, ud, yaw)
        return final_lr, final_fb, final_ud, 0
