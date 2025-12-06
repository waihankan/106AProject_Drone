from djitellopy import Tello
from interfaces.drone import IDrone
from utils.logger import setup_logger
import cv2
import threading
import time

logger = setup_logger("TelloDriver")

class TelloDrone(IDrone):
    def __init__(self):
        self.tello = Tello()
        self.frame_read = None

    def connect(self):
        logger.info("Connecting to Tello...")
        self.tello.connect()
        logger.info(f"Connected. Battery: {self.get_battery()}%")

    def disconnect(self):
        logger.info("Disconnecting...")
        self.stream_off()
        self.tello.end()

    def takeoff(self):
        logger.info("Taking off...")
        self.tello.takeoff()

    def land(self):
        logger.info("Landing...")
        self.tello.land()

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
