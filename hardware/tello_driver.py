from djitellopy import Tello
from interfaces.drone import IDrone
from utils.logger import setup_logger
import cv2
import threading
import time

logger = setup_logger("TelloDriver")

class TelloDrone(IDrone):
    def __init__(self, retry_count=3):
        self.tello = Tello()
        self.tello.retry_count = retry_count # Override default retry count (default is 3)
        self.frame_read = None
        self.flight_thread = None

    def connect(self):
        logger.info("Connecting to Tello...")
        try:
            self.tello.connect()
            logger.info(f"Connected. Battery: {self.get_battery()}%")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Tello: {e}")
            return False

    def disconnect(self):
        logger.info("Disconnecting...")
        self.stream_off()
        self.tello.end()

    def takeoff(self):
        if self.flight_thread and self.flight_thread.is_alive():
            logger.warning("Drone is busy (Takeoff/Land in progress). Ignoring command.")
            return
        
        logger.info("Taking off...")
        self.flight_thread = threading.Thread(target=self.tello.takeoff, daemon=True)
        self.flight_thread.start()

    def land(self):
        if self.flight_thread and self.flight_thread.is_alive():
            logger.warning("Drone is busy (Takeoff/Land in progress). Ignoring command.")
            return

        logger.info("Landing...")
        self.flight_thread = threading.Thread(target=self.tello.land, daemon=True)
        self.flight_thread.start()

    def stream_on(self):
        logger.info("Starting video stream...")
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()
        time.sleep(2) # Warmup

    def stream_off(self):
        logger.info("Stopping video stream...")
        self.tello.streamoff()

    def get_frame(self):
        if self.frame_read:
            return self.frame_read.frame
        return None

    def send_rc_control(self, left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity):
        self.tello.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)

    def get_battery(self) -> int:
        return self.tello.get_battery()
