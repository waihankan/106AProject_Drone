import cv2
import cv2.aruco as aruco
import numpy as np
import config
from interfaces.vision import IVision, VisionTarget
from utils.logger import setup_logger

logger = setup_logger("ArucoVision")

class ArucoVision(IVision):
    def __init__(self, dict_id=aruco.DICT_4X4_250):
        self.aruco_dict = aruco.getPredefinedDictionary(dict_id)
        self.parameters = aruco.DetectorParameters()
        
        # New OpenCV 4.7+ / 4.11 API
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def process(self, frame) -> list[VisionTarget]:
        if frame is None:
            return []

        # Workaround: Aruco detection fails on mirrored (flipped) images.
        # But User wants mirrored display.
        # Solution: Unflip for detection, then remap coordinates back.
        
        # 1. Unflip the frame to get "Real World" view for detection
        unflipped_frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(unflipped_frame, cv2.COLOR_BGR2GRAY)
        
        # 2. Detect on unflipped frame
        corners, ids, rejected = self.detector.detectMarkers(gray)
        # --- Pose estimation (FAKE calibration is OK for now) ---
        rvecs, tvecs = None, None
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                config.DRONE_MARKER_SIZE,
                config.DRONE_CAMERA_MATRIX,
                config.DRONE_DISTORTION_COEFFS,
            )

        
        # 3. Remap corners back to mirrored space if detected
        if ids is not None and len(corners) > 0:
            h, w = frame.shape[:2]
            for i in range(len(corners)):
                # corners[i] shape is (1, 4, 2) -> (batch, corners, xy)
                # Formula: x_mirrored = w - 1 - x_original
                corners[i][0, :, 0] = w - 1 - corners[i][0, :, 0]
        
        # 4. Draw on original (mirrored) frame with remapped corners
        if ids is not None:
             aruco.drawDetectedMarkers(frame, corners, ids)



        
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
                tvec = None
                rvec = None
                if tvecs is not None:
                    tvec = tvecs[i][0]   # shape (3,)
                    rvec = rvecs[i][0]

                targets.append(VisionTarget(
                    id=marker_id,
                    center=(norm_x, norm_y),
                    area=norm_area,
                    raw_corners=c,
                    tvec=tvec,
                    rvec=rvec,
                ))
        

        
        return targets
