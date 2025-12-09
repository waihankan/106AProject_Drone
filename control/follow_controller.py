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
        
        # Load from Config
        # If not present in config (legacy), default to 0.1
        try:
             import config
             self.dead_zone = config.DEADZONE_RATIO
        except:
             self.dead_zone = 0.1

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
        self.frames_since_lock = 0 # Buffer for FB command startup
        
        # Manual Calibration State
        self.is_calibrated = False
        self.calibration_frames = 0
        self.calibrated_area = None

    def _apply_ema(self, current, new, alpha):
        return (alpha * new) + ((1 - alpha) * current)

    def _apply_slew_limit(self, current_val, desired_val, max_change):
        diff = desired_val - current_val
        limited_diff = max(-max_change, min(max_change, diff))
        return int(current_val + limited_diff)

    def update(self, target: Optional[VisionTarget], current_state: dict, absolute_mode: bool = False) -> Tuple[int, int, int, int]:
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

        # 1. Update State (Smoothing & Buffer) - MOVED TO TOP
        if not self.has_locked:
             # First frame: Snap smoothers
             self.smooth_x = target.center[0]
             self.smooth_y = target.center[1]
             self.smooth_area = target.area
             self.has_locked = True
             self.frames_since_lock = 0
             
             # Also reset references if in relative mode
             if not absolute_mode:
                 self.ref_x = target.center[0]
                 self.ref_y = target.center[1]
                 self.ref_area = target.area
                 if hasattr(self, 'was_in_deadzone'): self.was_in_deadzone = False 
        else:
             # Apply Smoothers
             self.smooth_x = self._apply_ema(self.smooth_x, target.center[0], self.alpha_target)
             self.smooth_y = self._apply_ema(self.smooth_y, target.center[1], self.alpha_target)
             self.smooth_area = self._apply_ema(self.smooth_area, target.area, self.alpha_target)
             
             # Increment buffer
             if hasattr(self, 'frames_since_lock'):
                self.frames_since_lock += 1

        # 2. State-Specific Logic (Absolute vs Relative)
        if absolute_mode:
            try:
                import config
                self.ref_area = config.DESIRED_ARUCO_AREA
            except:
                self.ref_area = 0.05
            
            # Absolute Mode Reference is Always Center
            self.ref_x = 0.0
            self.ref_y = 0.0
            
            # --- Square Deadzone Logic ---
            ratio_x = self.dead_zone
            ratio_y = self.dead_zone * (16/9)
            
            # Check if In Deadzone
            in_deadzone = (abs(target.center[0]) < ratio_x) and (abs(target.center[1]) < ratio_y)
            
            # --- Calibration Logic ---
            if not self.is_calibrated:
                # User must hold marker in deadzone for 2 seconds (60 frames)
                if in_deadzone:
                    self.calibration_frames += 1
                    if self.calibration_frames >= 60:
                        self.is_calibrated = True
                        self.calibrated_area = self.smooth_area
                        logger.info(f"Calibration Complete. Reference Area: {self.calibrated_area:.4f}")
                else:
                    self.calibration_frames = 0
                
                # In calibration phase, NO MOVEMENT
                return 0, 0, 0, 0

            # --- Flight Logic (Calibrated) ---
            # Use the calibrated distance as reference
            if self.calibrated_area is not None:
                self.ref_area = self.calibrated_area
            
            # Deadzone Check for Flight (once calibrated)
            if in_deadzone:
                 delta_x = 0
                 delta_y = 0
            else:
                 delta_x = self.smooth_x - self.ref_x
                 delta_y = self.smooth_y - self.ref_y

            # Calculate Deltas (Using UPDATED smooth + UPDATED ref)
            delta_area = self.smooth_area - self.ref_area

        else:
            # Relative Mode (Hand)
            delta_x = self.smooth_x - self.ref_x
            delta_y = self.smooth_y - self.ref_y
            delta_area = self.smooth_area - self.ref_area
        
        # STRAFE Control (Left/Right)
        desired_lr = 0
        # In Absolute Mode: delta_x is 0 if in deadzone. So check is redundant but harmless.
        # In Relative Mode: we still need deadzone check?
        # Yes, Hand mode uses Config Deadzone too. 
        # But we didn't apply Square Logic there. Legacy behavior is fine for Hand.
        if absolute_mode:
             # Already handled deadzone (delta_x is 0 if in zone)
             desired_lr = int(delta_x * self.k_yaw)
        elif abs(delta_x) > self.dead_zone:
             desired_lr = int(delta_x * self.k_yaw) 

        # Height control (Up/Down)
        desired_ud = 0
        if absolute_mode:
             # Already handled
             desired_ud = int(-delta_y * self.k_throttle)
        elif abs(delta_y) > self.dead_zone:
            desired_ud = int(-delta_y * self.k_throttle)

        # PID for Distance (Forward/Backward)
        desired_fb = 0
        # Use a separate deadzone for area? Or fixed small value.
        AREA_DEADZONE = 0.005 # 0.5% tolerance
        # If in absolute deadzone (latched), delta_area might be small (tracking latched val).
        
        if abs(delta_area) > AREA_DEADZONE:
             # Inverted logic for Absolute Mode?
             if absolute_mode:
                 # User Request: "Larger should be +value". Consistent with Hand Mode.
                 # User Request: "Very slow to respond". Need higher gain for Aruco.
                 # Hand Mode uses * 20. Let's try * 40 for Aruco (Was 60, too sensitive).
                 desired_fb = int(delta_area * 60 * self.k_pitch)
             else:
                 # Relative Mode (Hand) - Positive logic (Closer = Positive Delta = Forward?)
                 # Hand: Big Hand = Closer. Delta Positive.
                 # If Hand is Close, we want to back away? 
                 # Legacy code says: delta_area * 20 * k. (Positive).
                 # If Hand Close -> Go Forward?? That seems wrong too.
                  # But User only complained about Aruco. Leave Hand alone.
                   if abs(delta_area) > 0.005: # Increased from 0.002 to reduce jitter sensitivity
                        desired_fb = int(delta_area * 20 * self.k_pitch)
                   
                   # Startup Buffer: Suppress FB for first 15 frames (0.5s) to avoid "Zoom on Pinch"
                   if hasattr(self, 'frames_since_lock') and self.frames_since_lock < 15:
                       desired_fb = 0
             
            
        # CONTROL PRIORITY LOGIC
        # Problem: FB (Distance) can be noisy and "win" the max_val check, preventing centering.
        # Solution: "Center First, Approach Later".
        
        # Thresholds for "Not Centered"
        CENTERING_THRESHOLD = 30 # Standard speed unit (max 100)
        
        # Check if we need significant centering
        needs_centering_lr = abs(desired_lr) > CENTERING_THRESHOLD
        needs_centering_ud = abs(desired_ud) > CENTERING_THRESHOLD
        
        if needs_centering_lr or needs_centering_ud:
            # We are far off-center. Focus on centering.
            # Suppress FB command significantly or entirely.
            # Let's dampen it to 0 so we don't drift forward while trying to turn.
            desired_fb = 0
            
        # WINNER TAKES ALL Logic (Secondary Filter)
        # Even after priority, we might still want to decouple axes if they are competing.
        abs_lr = abs(desired_lr)
        abs_fb = abs(desired_fb)
        abs_ud = abs(desired_ud)
        
        max_val = max(abs_lr, abs_fb, abs_ud)
        
        if max_val > 10: 
            if max_val == abs_lr:
                # Yaw wins
                desired_fb = 0
                desired_ud = 0
            elif max_val == abs_fb:
                # Pitch wins - only if we didn't already zero it above!
                desired_lr = 0
                desired_ud = 0
            elif max_val == abs_ud:
                # Throttle wins
                desired_lr = 0
                desired_fb = 0
        else:
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
