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
import config
from hardware.tello_driver import TelloDrone
from hardware.mock_driver import MockDrone
from vision.aruco_detector import ArucoVision
from vision.hand_gesture import HandGestureVision
from control.follow_controller import FollowController
from utils.logger import setup_logger

logger = setup_logger("Main")

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
    logger.info(f"Opening Control Camera (ID {config.WEBCAM_ID})...")
    control_cap = cv2.VideoCapture(config.WEBCAM_ID)
    if control_cap.isOpened():
            control_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            control_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    else:
        logger.error("Failed to open Control Camera!")

    controller = FollowController(target_id=0)

    try:
        # 4. Setup
        drone.connect()
        if config.ENABLE_DRONE_STREAM:
            drone.stream_on()
        
        logger.info("Battery: {}%".format(drone.get_battery()))
        logger.info("Press 't' to takeoff, 'l' to land, 'q' to quit.")

        is_flying = False
        last_frame_time = time.time()
        
        while True:
            # --- Stream Acquisition ---
            
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
            if control_cap and control_cap.isOpened():
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
                        cv2.putText(control_frame, "STOP COMMAND", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)

                # 6. Control Update
                # Calculate commands ALWAYS (for visualization)
                lr, fb, ud, yaw = controller.update(primary_target, {})
                
                if is_flying:
                    drone.send_rc_control(lr, fb, ud, yaw)
                    if control_frame is not None:
                        # Draw Active Commands (Green)
                        cv2.putText(control_frame, f"FLYING: LR:{lr} FB:{fb} UD:{ud}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    drone.send_rc_control(0, 0, 0, 0)
                    if control_frame is not None:
                        # Draw Preview Commands (Yellow/Blue)
                        cv2.putText(control_frame, f"STANDBY: LR:{lr} FB:{fb} UD:{ud}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            # --- Display ---
            if drone_frame is not None:
                cv2.imshow("Drone Stream", drone_frame)
            
            if control_frame is not None:
                cv2.imshow("Control View (Webcam)", control_frame)

            # --- Input ---
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('t'):
                if not is_flying:
                    drone.takeoff()
                    is_flying = True
            elif key == ord('l'):
                if is_flying:
                    drone.land()
                    is_flying = False

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
