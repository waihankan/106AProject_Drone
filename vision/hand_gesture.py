import cv2
import mediapipe as mp
import numpy as np
import config
from interfaces.vision import IVision, VisionTarget
from utils.logger import setup_logger

logger = setup_logger("HandGestureVision")

class HandGestureVision(IVision):
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        # min_detection_confidence=0.7 for higher precision
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
    def _calculate_distance(self, p1, p2):
        return ((p1.x - p2.x)**2 + (p1.y - p2.y)**2)**0.5

    def process(self, frame) -> list[VisionTarget]:
        if frame is None:
            return []

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        targets = []
        h, w, _ = frame.shape

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # --- Landmark Extraction ---
                wrist = hand_landmarks.landmark[0]
                thumb_tip = hand_landmarks.landmark[4]
                index_tip = hand_landmarks.landmark[8]
                index_mcp = hand_landmarks.landmark[5]
                middle_mcp = hand_landmarks.landmark[9]
                pinky_mcp = hand_landmarks.landmark[17]

                # --- Robust Scale Calculation (Palm Size) ---
                # 1. Palm Height (Wrist to Middle Knuckle) - Robust to Yaw
                palm_height = self._calculate_distance(wrist, middle_mcp)
                
                # 2. Palm Width (Index Knuckle to Pinky Knuckle) - Robust to Pitch
                palm_width = self._calculate_distance(index_mcp, pinky_mcp)
                
                # 3. Robust Scale: Max of both. 
                robust_size = max(palm_height, palm_width)
                area = robust_size

                # --- Pinch Detection ---
                # Calculate pinch distance
                dist = self._calculate_distance(thumb_tip, index_tip)
                
                # Dynamic Pinch Threshold (Distance < Ratio * PalmSize)
                pinch_threshold = max(robust_size * config.HAND_PINCH_RATIO, 0.02)
                is_pinched = dist < pinch_threshold
                
                # --- Control Point Calculation ---
                # Center point is midpoint of pinch
                cx = (thumb_tip.x + index_tip.x) / 2
                cy = (thumb_tip.y + index_tip.y) / 2
                
                # Normalize (-1 to 1) for control output
                norm_x = (2 * cx) - 1
                norm_y = (2 * cy) - 1

                # --- Visualization ---
                px_t = (int(thumb_tip.x * w), int(thumb_tip.y * h))
                px_i = (int(index_tip.x * w), int(index_tip.y * h))
                px_c = (int(cx * w), int(cy * h))
                
                color = (0, 255, 0) if is_pinched else (0, 0, 255) # Green if pinch, Red if open
                
                cv2.line(frame, px_t, px_i, color, 3)
                cv2.circle(frame, px_c, 5, color, -1)
                
                if is_pinched:
                    # PINCHED = FOLLOW (ID 0)
                    targets.append(VisionTarget(
                        id=0,
                        center=(norm_x, norm_y),
                        area=area,
                        raw_corners=None
                    ))
                    cv2.putText(frame, "PINCH: MOVING", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, "OPEN: HOVERING", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Draw skeleton
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        return targets
