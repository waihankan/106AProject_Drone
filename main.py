import os
import sys

# Suppress standard logging (MediaPipe/TensorFlow C++ logs)
# MUST PROCEED other imports
os.environ['GLOG_minloglevel'] = '2'       # Suppress GLOG info/warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'   # Suppress TensorFlow info/warnings

import warnings
# Suppress Protobuf DeprecationWarning
warnings.filterwarnings("ignore", category=UserWarning, module="google.protobuf.symbol_database")

import time
import argparse
import cv2
import numpy as np
import config
from hardware.tello_driver import TelloDrone
from hardware.mock_driver import MockDrone
from vision.aruco_detector import ArucoVision
from vision.hand_gesture import HandGestureVision
from control.follow_controller import FollowController
from utils.logger import setup_logger

logger = setup_logger("Main")

def draw_hud(frame, battery, is_flying, current_commands=None):
    """Draws Heads-Up Display (HUD) with transparent overlays."""
    if frame is None: return

    h, w, _ = frame.shape
    overlay = frame.copy()
    
    # Colors & Fonts
    color_white = (255, 255, 255)
    color_green = (0, 255, 0)
    color_red = (0, 0, 255)
    color_cyan = (255, 255, 0)
    color_bg = (0, 0, 0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # --- Top Bar (Status) ---
    # Background strip
    cv2.rectangle(overlay, (0, 0), (w, 40), color_bg, -1)
    
    # Battery Bar (Top Right)
    bar_width = 100
    bar_height = 15
    bar_x = w - bar_width - 20
    bar_y = 12
    # Outline
    cv2.rectangle(overlay, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), color_white, 1)
    # Fill
    fill_width = int(bar_width * (battery / 100.0))
    bat_color = color_green if battery > 20 else color_red
    cv2.rectangle(overlay, (bar_x, bar_y), (bar_x + fill_width, bar_y + bar_height), bat_color, -1)
    # Text
    cv2.putText(overlay, f"{battery}%", (bar_x - 45, bar_y + 12), font, 0.5, color_white, 1)

    # Info Text (Top Left)
    info_text = f"HW: {config.HARDWARE_MODE.upper()} | VIS: {config.VISION_MODE.upper()}"
    cv2.putText(overlay, info_text, (20, 25), font, 0.6, color_white, 1)
    
    # Flight State (Top Center)
    state_text = "FLYING" if is_flying else "STANDBY"
    state_color = color_green if is_flying else color_red
    text_size = cv2.getTextSize(state_text, font, 0.8, 2)[0]
    cv2.putText(overlay, state_text, ((w - text_size[0]) // 2, 28), font, 0.8, state_color, 2)

    # --- Crosshair ---
    cx, cy = w // 2, h // 2
    cv2.line(overlay, (cx - 20, cy), (cx + 20, cy), color_white, 1)
    cv2.line(overlay, (cx, cy - 20), (cx, cy + 20), color_white, 1)
    cv2.circle(overlay, (cx, cy), 10, color_white, 1)
    
    # --- Bottom Panel (Commands) ---
    if current_commands:
        # Transparent Box at bottom left
        cv2.rectangle(overlay, (20, h - 60), (350, h - 20), color_bg, -1)
        lr, fb, ud, yaw = current_commands
        cmd_text = f"CMD: LR:{lr} FB:{fb} UD:{ud} Y:{yaw}"
        cv2.putText(overlay, cmd_text, (30, h - 35), font, 0.6, color_cyan, 1)
        
    # Apply Transparency
    alpha = 0.6 # Transparency factor
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

def main():
    # Parse args to override config (optional)
    parser = argparse.ArgumentParser(description='Drone Control System')
    parser.add_argument('--hardware', type=str, help='Override hardware mode')
    parser.add_argument('--vision', type=str, help='Override vision mode')
    args = parser.parse_args()

    # Apply overrides
    if args.hardware: config.HARDWARE_MODE = args.hardware
    if args.vision: config.VISION_MODE = args.vision
    
    # Auto-select settings could go here
    # e.g. force hardware=mock if no drone found, etc.
    pass

    logger.info(f"System Starting: HW={config.HARDWARE_MODE}, VIS={config.VISION_MODE}, ENABLE_STREAM={config.ENABLE_DRONE_STREAM}")

    # 1. Initialize Drone (The thing that Flies)
    if config.HARDWARE_MODE == 'mock':
        drone = MockDrone()
    else:
        drone = TelloDrone()

    # 2. Initialize Vision (The thing that Sees)
    if config.VISION_MODE == 'hand':
        vision = HandGestureVision()
    else:
        vision = ArucoVision()
    
    # 3. Initialize Control Source (Webcam)
    # If using MOCK hardware, do NOT open webcam 0 again (MockDrone already has it).
    control_cap = None
    if config.HARDWARE_MODE != 'mock':
        logger.info(f"Opening Control Camera (ID {config.WEBCAM_ID})...")
        control_cap = cv2.VideoCapture(config.WEBCAM_ID)
        if control_cap.isOpened():
                control_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                control_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        else:
            logger.error("Failed to open Control Camera!")
    else:
        logger.info("Mock Mode: Using simulated drone video for control (Shared Webcam).")

    controller = FollowController(target_id=0)

    try:
        # 4. Setup
        drone.connect()
        if config.ENABLE_DRONE_STREAM:
            drone.stream_on()
        
        # Initial Battery
        battery_level = drone.get_battery()
        last_battery_time = time.time()
        
        logger.info("Battery: {}%".format(battery_level))
        logger.info("Press 't' to takeoff, 'l' to land, 'q' to quit.")

        is_flying = False
        last_frame_time = time.time()
        
        while True:
            # --- Stream Acquisition ---
            
            # Update Battery periodically (every 10s)
            if time.time() - last_battery_time > 10.0:
                 # Run in thread or just quick read? SDK get_battery is usually blocking 
                 # but fast enough. For safety, let's keep it simple for now or skip if busy.
                 # Tello SDK reads from last stats packet, so it should be non-blocking instant.
                 try:
                     battery_level = drone.get_battery()
                     last_battery_time = time.time()
                 except:
                     pass

            # A. Drone Stream (FPV)
            drone_frame = None
            if config.ENABLE_DRONE_STREAM:
                drone_frame = drone.get_frame()
                
                # Watchdog Logic
                if config.HARDWARE_MODE == 'tello' and drone_frame is None:
                     if time.time() - last_frame_time > config.WATCHDOG_TIMEOUT:
                        logger.warning("DRONE VIDEO LOST!")
                
                if drone_frame is not None:
                    last_frame_time = time.time()
                    # Optional visual resize
                    # drone_frame = cv2.resize(drone_frame, (960, 720))

            # B. Control Stream (Webcam)
            control_frame = None
            
            if config.HARDWARE_MODE == 'mock':
                # In Mock mode, the "Drone" IS the webcam. Use it for control.
                # Use get_frame() directly.
                frame = drone.get_frame()
                if frame is not None:
                     control_frame = frame.copy()
                     # Mock driver already flips it? Let's check. 
                     # Usually webcam raw is not flipped.
                     control_frame = cv2.flip(control_frame, 1) 

            elif control_cap and control_cap.isOpened():
                ret, frame = control_cap.read()
                
                if ret:
                    control_frame = frame
                    # Flip webcam for intuitive mirror interaction
                    control_frame = cv2.flip(control_frame, 1) 

            # --- Logic ---
            
            # 5. Vision Processing
            # Only process if we have a frame to process
            targets = []
            if control_frame is not None:
                targets = vision.process(control_frame)
                
                # Find Target
                primary_target = None
                for t in targets:
                     # Aruco ID 0 or Hand Gesture ID 0 (Follow)
                    if t.id == 0:
                        primary_target = t
                        break
                    # Hand Gesture ID 1 (Stop)
                    elif t.id == 1 and config.VISION_MODE == 'hand':
                        primary_target = None 
                        cv2.putText(control_frame, "STOP COMMAND", (300, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)

                # 6. Control Update
                # Calculate commands ALWAYS (for visualization)
                lr, fb, ud, yaw = controller.update(primary_target, {})
                
                # 7. Draw HUD
                current_cmds = (lr, fb, ud, yaw)
                draw_hud(control_frame, battery_level, is_flying, current_cmds)
                
                if drone_frame is not None:
                     # For drone frame, we might not want to reprint commands if they overwrite interesting stuff,
                     # but for consistency let's show them.
                     draw_hud(drone_frame, battery_level, is_flying, current_cmds)

            # --- Display ---
            window_shown = False
            if drone_frame is not None:
                cv2.imshow("Drone Stream", drone_frame)
                window_shown = True
            
            if control_frame is not None:
                cv2.imshow("Control View (Webcam)", control_frame)
                window_shown = True

            # --- Input ---
            # Ensure at least one window exists for waitKey to work!
            if not window_shown:
                # Create a black dummy window for status/input
                dummy = 255 * np.ones((400, 600, 3), dtype=np.uint8) 
                draw_hud(dummy, battery_level, is_flying)
                cv2.putText(dummy, "NO VIDEO", (200, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow("Drone Control Panel", dummy)

            key = cv2.waitKey(1) & 0xFF
            
            # Global Keys
            if key == ord('q'):
                break
            elif key == ord('t') or key == ord('T'):
                if not is_flying:
                    drone.takeoff()
                    is_flying = True
            elif key == ord('l') or key == ord('L'):
                if is_flying:
                    drone.land()
                    is_flying = False
            
            # Manual Control Override (Emergency)
            # Hold 'm' to enable manual control (stops vision control)
            # This is simple; for toggle, we'd need a state var.
            # Let's use specific keys that OVERRIDE calculated logic for this frame.
            
            manual_input = False
            man_lr, man_fb, man_ud, man_yaw = 0, 0, 0, 0
            
            # WASD for Movement (Right Hand Mode 2 equivalent-ish)
            if key == ord('w'): # Forward
                man_fb = 50
                manual_input = True
            elif key == ord('s'): # Backward
                man_fb = -50
                manual_input = True
            elif key == ord('a'): # Left
                man_lr = -50
                manual_input = True
            elif key == ord('d'): # Right
                man_lr = 50
                manual_input = True
            
            # Arrow Keys (Up/Down/Yaw) - Map to I/K/J/L for easier keyboard access
            elif key == ord('i'): # Up
                man_ud = 50
                manual_input = True
            elif key == ord('k'): # Down
                man_ud = -50
                manual_input = True
            elif key == ord('j'): # Yaw Left
                man_yaw = -50
                manual_input = True
            elif key == ord('l'): # Yaw Right causes conflict with Land? 'l' is Land. 
                # Let's map Land to 'space'? Or Yaw Right to 'u'.
                # Let's stick to simple WASD + Up/Down (I/K) for now.
                pass
            
            if manual_input:
                # Override Vision Commands
                lr, fb, ud, yaw = man_lr, man_fb, man_ud, man_yaw
                if control_frame is not None:
                     cv2.putText(control_frame, "MANUAL OVERRIDE", (300, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)

            # Send FINAL command (Vision or Manual)
            if is_flying:
                drone.send_rc_control(lr, fb, ud, yaw)
            else:
                drone.send_rc_control(0, 0, 0, 0)

    except KeyboardInterrupt:
        logger.info("Keyboard Interrupt detected.")
    except Exception as e:
        logger.error(f"An error occurred: {e}")
    finally:
        if is_flying:
             drone.land()
        drone.disconnect()
        if control_cap:
            control_cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
