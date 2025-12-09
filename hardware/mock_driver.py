import cv2
import numpy as np
import time
from interfaces.drone import IDrone
from utils.logger import setup_logger

logger = setup_logger("MockDriver")

class MockDrone(IDrone):
    def __init__(self):
        self.cap = None
        self.battery = 100
        self.is_connected = False

    def connect(self):
        logger.info("MOCK: Connecting to Webcam...")
        self.cap = cv2.VideoCapture(0)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.is_connected = True
        if not self.cap.isOpened():
            logger.error("MOCK: Failed to open webcam!")

    def disconnect(self):
        logger.info("MOCK: Disconnecting...")
        if self.cap:
            self.cap.release()
        self.is_connected = False

    def takeoff(self):
        logger.info("MOCK: TAKEOFF COMMAND RECEIVED")
        self.battery -= 1

    def land(self):
        logger.info("MOCK: LAND COMMAND RECEIVED")

    def stream_on(self):
        logger.info("MOCK: Video Stream Started")
        
    def stream_off(self):
        logger.info("MOCK: Video Stream Stopped")

    def get_frame(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                return frame
            else:
                logger.warning("MOCK: Failed to read frame")
        return None

    def send_rc_control(self, left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity):
        # Only log if there's actual movement to avoid spamming 0s
        if any([left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity]):
            logger.info(f"MOCK RC: LR={left_right_velocity} FB={forward_backward_velocity} UD={up_down_velocity} YAW={yaw_velocity}")
        # pass

    def get_battery(self) -> int:
        return self.battery
