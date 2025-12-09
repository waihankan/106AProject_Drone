import unittest
from control.follow_controller import FollowController
from interfaces.vision import VisionTarget
import config

# Mock Config
config.DEADZONE_RATIO = 0.1
config.DESIRED_ARUCO_AREA = 0.05
config.K_PITCH = 50.0 # Standard

class TestFollowController(unittest.TestCase):
    def setUp(self):
        self.controller = FollowController(target_id=0)
        self.controller.k_yaw = 100
        self.controller.k_throttle = 100
        self.controller.k_pitch = 50
        
    def test_absolute_mode_deadzone(self):
        # Target in deadzone (0.05 < 0.1)
        target = VisionTarget(id=0, center=(0.05, 0.05), area=0.05, raw_corners=[])
        
        lr, fb, ud, yaw = self.controller.update(target, {}, absolute_mode=True)
        
        self.assertEqual(lr, 0, "Yaw should be 0 inside deadzone")
        self.assertEqual(ud, 0, "Throttle should be 0 inside deadzone")
        self.assertEqual(fb, 0, "Pitch should be 0 for exact area match")

    def test_absolute_mode_yaw_sign(self):
        # Target to Right (0.5). Drone should Turn Right (+Yaw).
        target = VisionTarget(id=0, center=(0.5, 0.0), area=0.05, raw_corners=[])
        
        lr, fb, ud, yaw = self.controller.update(target, {}, absolute_mode=True)
        print(f"Yaw Test: Target=0.5, Output={lr}")
        self.assertTrue(lr > 0, "Should turn POSITIVE (Right) when target is Right")

    def test_absolute_mode_throttle_sign(self):
        # Target Below Center (Positive 0.5 in CV coordinates). 
        # Drone should go DOWN (-Throttle).
        # Wait. Y: Top=-1, Bottom=1.
        # If target is at 0.5 (Bottom half). We need to go DOWN.
        # Controller Logic: desired_ud = -delta_y * k.
        # delta_y = 0.5 - 0 = 0.5.
        # desired = -0.5 * k -> Negative. Correct.
        target = VisionTarget(id=0, center=(0.0, 0.5), area=0.05, raw_corners=[])
        
        lr, fb, ud, yaw = self.controller.update(target, {}, absolute_mode=True)
        print(f"Throttle Test: Target=0.5 (Low), Output={ud}")
        self.assertTrue(ud < 0, "Should go NEGATIVE (Down) when target is Low")

    def test_absolute_mode_distance_too_close(self):
        # Target Too Close (Area 0.1 > Ref 0.05).
        # Should go BACKWARD (Negative).
        target = VisionTarget(id=0, center=(0.0, 0.0), area=0.1, raw_corners=[])
        
        lr, fb, ud, yaw = self.controller.update(target, {}, absolute_mode=True)
        print(f"Distance Too Close Test: Area=0.1 (Ref=0.05), Output={fb}")
        self.assertTrue(fb < 0, "Should go NEGATIVE (Backward) when too close")

    def test_absolute_mode_distance_too_far(self):
        # Target Too Far (Area 0.01 < Ref 0.05).
        # Should go FORWARD (Positive).
        target = VisionTarget(id=0, center=(0.0, 0.0), area=0.01, raw_corners=[])
        
        lr, fb, ud, yaw = self.controller.update(target, {}, absolute_mode=True)
        print(f"Distance Too Far Test: Area=0.01 (Ref=0.05), Output={fb}")
        self.assertTrue(fb > 0, "Should go POSITIVE (Forward) when too far")

if __name__ == '__main__':
    unittest.main()
