import cv2
import cv2.aruco as aruco
import numpy as np
from interfaces.vision import IVision, VisionTarget
from utils.logger import setup_logger

logger = setup_logger("ArucoVision")

class ArucoVision(IVision):
    def __init__(self, dict_id=aruco.DICT_4X4_50):
        self.aruco_dict = aruco.getPredefinedDictionary(dict_id)
        self.parameters = aruco.DetectorParameters()
        # Older OpenCV versions might need:
        # self.parameters = aruco.DetectorParameters_create()

    def process(self, frame) -> list[VisionTarget]:
        if frame is None:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        targets = []
        h, w, _ = frame.shape

        if ids is not None:
            # aruco.drawDetectedMarkers(frame, corners, ids) # Optional: Detection visualization can be done outside or here if we want to modify frame (bad practice for this interface, better to have a 'draw' method)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                c = corners[i][0]
                
                # Calculate center
                cx = np.mean(c[:, 0])
                cy = np.mean(c[:, 1])
                
                # Normalize center (-1 to 1)
                # (cx - w/2) / (w/2)  => 2*cx/w - 1
                norm_x = (2 * cx / w) - 1
                norm_y = (2 * cy / h) - 1
                
                # Calculate approximated area/distance
                # Simply using area of the bounding box
                area = cv2.contourArea(c)
                norm_area = area / (w * h)

                targets.append(VisionTarget(
                    id=marker_id,
                    center=(norm_x, norm_y),
                    area=norm_area,
                    raw_corners=c
                ))
        
        return targets
