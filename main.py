import cv2
import time
from hardware.tello_driver import TelloDrone
from vision.aruco_detector import ArucoVision
from control.follow_controller import FollowController
from utils.logger import setup_logger

logger = setup_logger("Main")

def main():
    # 1. Initialize Components
    drone = TelloDrone()
    vision = ArucoVision()
    controller = FollowController(target_id=0)

    try:
        # 2. Setup
        drone.connect()
        drone.stream_on()
        
        logger.info("Battery: {}%".format(drone.get_battery()))
        logger.info("Press 't' to takeoff, 'l' to land, 'q' to quit.")

        is_flying = False

        last_frame_time = time.time()
        
        while True:
            # 3. Get Frame
            frame = drone.get_frame()
            if frame is None:
                if time.time() - last_frame_time > 2.0:
                    logger.warning("CONNECTION LOST: No frame for 2 seconds!")
                    if is_flying:
                        logger.warning("Initiating Emergency Landing due to signal loss.")
                        drone.land()
                        is_flying = False # Prevent further commands
                time.sleep(0.01)
                continue
            
            # Reset watchdog
            last_frame_time = time.time()

            # 4. Resize for performance (optional, Tello is usually 960x720)
            frame = cv2.resize(frame, (640, 480))

            # 5. Vision Processing
            targets = vision.process(frame)
            
            # Find primary target
            primary_target = None
            for t in targets:
                if t.id == 0:
                    primary_target = t
                    # Draw visual verification
                    h, w, _ = frame.shape
                    cx = int((t.center[0] + 1) * w / 2)
                    cy = int((t.center[1] + 1) * h / 2)
                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                    break

            # 6. Control Logic
            # Only send commands if we are actually enabled/flying to prevent accidents on ground
            if is_flying:
                lr, fb, ud, yaw = controller.update(primary_target, {})
                drone.send_rc_control(lr, fb, ud, yaw)
                cv2.putText(frame, f"CMD: {lr} {fb} {ud} {yaw}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                drone.send_rc_control(0, 0, 0, 0)
                cv2.putText(frame, "STANDBY", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            # 7. Display
            cv2.imshow("Drone View", frame)

            # 8. User Input
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
        # 9. Cleanup
        if is_flying:
            logger.info("Emergency Landing...")
            drone.land()
        drone.disconnect()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
