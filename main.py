import os
import sys

# Suppress standard logging (MediaPipe/TensorFlow C++ logs)
# MUST PROCEED other imports
os.environ["GLOG_minloglevel"] = "2"  # Suppress GLOG info/warnings
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"  # Suppress TensorFlow info/warnings

import warnings

# Suppress Protobuf DeprecationWarning
warnings.filterwarnings(
    "ignore", category=UserWarning, module="google.protobuf.symbol_database"
)

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
from utils.followMe import (
    generate_spiral_waypoints,
    rc_to_reach_tvec,
    clamp,
    reached_waypoint,
)

logger = setup_logger("Main")


def draw_hud(
    frame,
    battery,
    is_flying,
    current_commands=None,
    primary_target=None,
    controller=None,
    show_deadzone=True,
):
    """Draws Heads-Up Display (HUD) with transparent overlays."""
    if frame is None:
        return

    h, w, _ = frame.shape
    overlay = frame.copy()

    # Colors & Fonts
    color_white = (255, 255, 255)
    color_green = (0, 255, 0)
    color_red = (0, 0, 255)
    color_cyan = (255, 255, 0)
    color_orange = (0, 165, 255)
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
    cv2.rectangle(
        overlay, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), color_white, 1
    )
    # Fill
    fill_width = int(bar_width * (battery / 100.0))
    bat_color = color_green if battery > 20 else color_red
    cv2.rectangle(
        overlay, (bar_x, bar_y), (bar_x + fill_width, bar_y + bar_height), bat_color, -1
    )
    # Text
    cv2.putText(
        overlay, f"{battery}%", (bar_x - 45, bar_y + 12), font, 0.5, color_white, 1
    )

    # Info Text (Top Left)
    info_text = (
        f"HW: {config.HARDWARE_MODE.upper()} | VIS: {config.VISION_MODE.upper()}"
    )
    cv2.putText(overlay, info_text, (20, 25), font, 0.6, color_white, 1)

    # Flight State (Top Center)
    state_text = "FLYING" if is_flying else "STANDBY"
    state_color = color_green if is_flying else color_red
    text_size = cv2.getTextSize(state_text, font, 0.8, 2)[0]
    cv2.putText(
        overlay, state_text, ((w - text_size[0]) // 2, 28), font, 0.8, state_color, 2
    )

    # --- Target Status ---
    # Centered just below top bar
    if primary_target:
        status_msg = f"LOCKED: ID {primary_target.id}"
        status_color = color_green
        ts_size = cv2.getTextSize(status_msg, font, 0.7, 2)[0]
        cv2.putText(
            overlay, status_msg, ((w - ts_size[0]) // 2, 70), font, 0.7, status_color, 2
        )
    elif config.VISION_MODE == "aruco":
        # Only show searching text for Aruco
        status_msg = "NO TARGET - SEARCHING..."
        status_color = color_red
        ts_size = cv2.getTextSize(status_msg, font, 0.7, 2)[0]
        cv2.putText(
            overlay, status_msg, ((w - ts_size[0]) // 2, 70), font, 0.7, status_color, 2
        )

    # --- Deadzone Visual ---
    # Draw Red Box for Deadzone (Only in Absolute/Aruco Mode)
    if show_deadzone and config.VISION_MODE == "aruco":
        try:
            dz = config.DEADZONE_RATIO  # e.g. 0.3
        except:
            dz = 0.2

        # Calculate box coordinates from center
        cx, cy = w // 2, h // 2

        # Aspect Ratio logic to match Controller
        # Controller uses: ratio_y = ratio_x * (16/9).
        # But wait, Normalized Coordinates means:
        # X=1 is Edge. Y=1 is Edge.
        # If I draw a box 0.3 * Width and 0.3 * Height.
        # Width=1280 (0.3->384). Height=720 (0.3->216).
        # 384 vs 216.
        # Physical Ratio = 384/216 = 1.77.
        # So a "Normal" normalized box IS ALREADY physically 16:9 RECTANGLE.
        # If we want a SQUARE Physical Box using Normalized Coordinates:
        # We need Physical Width == Physical Height.
        # W_px = 2 * nx * (W/2) = nx * W
        # H_px = 2 * ny * (H/2) = ny * H
        # We want W_px = H_px.
        # nx * W = ny * H
        # ny = nx * (W/H).
        # ny = 0.3 * (16/9) = 0.533.
        # Normalized Y threshold needs to be LARGER.

        ratio_x = dz
        ratio_y = dz * (w / h)

        # Let's verify Squareness.
        # If W=1600, H=900. ratio_x=0.2. ratio_y=0.2*1.77=0.355.
        # dx = 800 * 0.2 = 160.
        # dy = 450 * 0.355 = 159.75.
        # dx ~= dy. It IS Square!

        # Pixel calculation:
        d_px_x = int((w / 2) * ratio_x)
        d_px_y = int((h / 2) * ratio_y)

        # Top-Left, Bottom-Right
        p1 = (cx - d_px_x, cy - d_px_y)
        p2 = (cx + d_px_x, cy + d_px_y)

        # --- Calibration UI ---
        # Check Calibration State from Controller
        is_calibrated = getattr(
            controller, "is_calibrated", True
        )  # Default True for safety if attr missing

        if not is_calibrated:
            # Flashing Box (Red/Yellow)
            import time

            flash = (int(time.time() * 5) % 2) == 0
            box_color = (0, 255, 255) if flash else (0, 0, 255)  # Yellow/Red flash

            thickness = 3
            cv2.rectangle(overlay, p1, p2, box_color, thickness)

            # Calibration Instruction Text
            calib_msg = "HOLD ARUCO TAG IN BOX TO CALIBRATE"
            calib_color = (0, 255, 255)
            c_size = cv2.getTextSize(calib_msg, font, 0.8, 2)[0]
            cv2.putText(
                overlay,
                calib_msg,
                ((w - c_size[0]) // 2, cy - d_px_y - 20),
                font,
                0.8,
                calib_color,
                2,
            )

            # Progress Bar?
            calib_frames = getattr(controller, "calibration_frames", 0)
            if calib_frames > 0:
                progress = calib_frames / 60.0
                # Draw bar below box
                bar_w = int((d_px_x * 2) * progress)
                cv2.rectangle(
                    overlay,
                    (p1[0], p2[1] + 10),
                    (p1[0] + bar_w, p2[1] + 20),
                    (0, 255, 0),
                    -1,
                )

        else:
            # Normal Operation (Solid Green Box? Or Red?)
            # User likes Red Box for Deadzone.
            # Maybe Green if "Locked/In Zone"?
            # Let's keep it Red/Green based on entry?
            # User didn't ask for that. Keep standard Red.
            cv2.rectangle(overlay, p1, p2, (0, 0, 255), 2)

            # Optional: Show "CALIBRATED" briefly?

    # --- Bottom Panel (Commands) ---
    if current_commands:
        # Transparent Box at bottom left
        cv2.rectangle(overlay, (20, h - 60), (350, h - 20), color_bg, -1)
        lr, fb, ud, yaw = current_commands
        cmd_text = f"CMD: LR:{lr} FB:{fb} UD:{ud} Y:{yaw}"
        cv2.putText(overlay, cmd_text, (30, h - 35), font, 0.6, color_cyan, 1)

    # Apply Transparency
    alpha = 0.6  # Transparency factor
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)


def main():
    # Parse args to override config (optional)
    parser = argparse.ArgumentParser(description="Drone Control System")
    parser.add_argument("--hardware", type=str, help="Override hardware mode")
    parser.add_argument("--vision", type=str, help="Override vision mode")
    args = parser.parse_args()

    # Apply overrides
    if args.hardware:
        config.HARDWARE_MODE = args.hardware
    if args.vision:
        config.VISION_MODE = args.vision

    # Auto-select settings could go here
    # e.g. force hardware=mock if no drone found, etc.
    pass

    logger.info(
        f"System Starting: HW={config.HARDWARE_MODE}, VIS={config.VISION_MODE}, ENABLE_STREAM={config.ENABLE_DRONE_STREAM}"
    )

    # 1. Initialize Drone (The thing that Flies)
    if config.HARDWARE_MODE == "mock":
        drone = MockDrone()
    else:
        drone = TelloDrone(retry_count=1)

    # 2. Initialize Vision (The thing that Sees)
    if config.VISION_MODE == "hand":
        vision = HandGestureVision()
    else:
        vision = ArucoVision()

    # 3. Initialize Control Source (Webcam)
    # If using MOCK hardware, do NOT open webcam 0 again (MockDrone already has it).
    control_cap = None
    if config.HARDWARE_MODE != "mock":
        logger.info(f"Opening Control Camera (ID {config.WEBCAM_ID})...")
        control_cap = cv2.VideoCapture(config.WEBCAM_ID)
        if control_cap.isOpened():
            control_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            control_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        else:
            logger.error("Failed to open Control Camera!")
    else:
        logger.info(
            "Mock Mode: Using simulated drone video for control (Shared Webcam)."
        )

    # Determine Target ID based on Vision Mode
    # Hand Gesture is hardcoded to ID 0. Aruco uses Config (User=2).
    active_target_id = config.TARGET_ID if config.VISION_MODE == "aruco" else 0
    controller = FollowController(target_id=active_target_id)

    try:
        # 4. Setup

        # Show Splash Screen to indicate connection attempt
        if config.HARDWARE_MODE == "tello":
            splash = np.zeros((400, 800, 3), dtype=np.uint8)
            cv2.putText(
                splash,
                "CONNECTING TO TELLO...",
                (50, 200),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                splash,
                "Please ensure WiFi is connected.",
                (50, 250),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            cv2.imshow("Drone Control Panel", splash)
            cv2.waitKey(100)

        # Try to connect
        connected = False
        while not connected:
            success = drone.connect()
            if success:
                connected = True
            else:
                # Show Error Screen
                if config.HARDWARE_MODE == "tello":
                    splash = np.zeros((400, 800, 3), dtype=np.uint8)
                    cv2.putText(
                        splash,
                        "CONNECTION FAILED!",
                        (50, 150),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                        2,
                    )
                    cv2.putText(
                        splash,
                        "1. Check WiFi Connection",
                        (50, 220),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )
                    cv2.putText(
                        splash,
                        "2. Check Firewall / VPN",
                        (50, 250),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )
                    cv2.putText(
                        splash,
                        "3. Press 'r' to Retry, 'q' to Quit",
                        (50, 300),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2,
                    )
                    cv2.imshow("Drone Control Panel", splash)

                    key = cv2.waitKey(0) & 0xFF
                    if key == ord("q"):
                        print("Quitting...")
                        return
                    elif key == ord("r"):
                        cv2.putText(
                            splash,
                            "RETRYING...",
                            (50, 350),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2,
                        )
                        cv2.imshow("Drone Control Panel", splash)
                        cv2.waitKey(100)
                        continue
                else:
                    break  # Should not happen in mock mode, but break to avoid loop
        if config.ENABLE_DRONE_STREAM:
            drone.stream_on()

        # Initial Battery
        battery_level = drone.get_battery()
        last_battery_time = time.time()

        logger.info("Battery: {}%".format(battery_level))
        logger.info("Press 't' to takeoff, 'l' to land, 'q' to quit.")

        is_flying = False
        last_frame_time = time.time()

        SEARCH_YAW_DPS = 30  # yaw speed while rotating
        SEARCH_STEP_DEG = 30  # rotate this many degrees each step
        SEARCH_PAUSE_S = 1 # pause this long after each step to search

        search_phase = "ROTATE"  # "ROTATE" or "PAUSE"
        search_enabled = False
        search_status = "SEARCH DISABLED"

        numOfWP = 5
        waypoints = []
        wp_idx = 0
        STOP_DIST = 0.5  # meters
        search_idx = 0
        
        LOST_FRAMES_TO_SEARCH = 8
        found_frames = 0
        lost_frames = 0

        while True:
            window_shown = False  # Initialization for loop
            # status for on-screen display (drone window)
            # e.g. "YAWING (ROTATE)" / "SEARCH PAUSE" / "MARKER FOUND"

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

                if config.HARDWARE_MODE == "tello" and drone_frame is None:
                    if time.time() - last_frame_time > config.WATCHDOG_TIMEOUT:
                        logger.warning("DRONE VIDEO LOST!")

                if drone_frame is not None:
                    last_frame_time = time.time()
                    # Optional visual resize
                    # drone_frame = cv2.resize(drone_frame, (960, 720))

            # B. Control Stream (Webcam)
            control_frame = None

            if config.HARDWARE_MODE == "mock":
                # In Mock mode, the "Drone" IS the webcam. Use it for control.
                # Use get_frame() directly.
                frame = drone.get_frame()
                if frame is not None:
                    control_frame = frame.copy()
                    # Mock driver already flips it? Let's check.
                    # Usually webcam raw is not flipped.
                    control_frame = cv2.flip(control_frame, 1)
                    # In mock mode expose the raw (unflipped) webcam frame as drone_frame
                    # so later drone_display = cv2.flip(drone_frame, 1) matches control_frame.
                    drone_frame = frame.copy()

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
                    if t.id == active_target_id:
                        primary_target = t
                        break
                    # Hand Gesture ID 1 (Stop)
                    elif t.id == 1 and config.VISION_MODE == "hand":
                        primary_target = None
                        cv2.putText(
                            control_frame,
                            "STOP COMMAND",
                            (300, 50),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 0, 255),
                            3,
                        )

                # 6. Control Update
                # Calculate commands ALWAYS (for visualization)
                # Enable Absolute Mode (Center Tracking) if using Aruco
                is_absolute = config.VISION_MODE == "aruco"
                lr, fb, ud, yaw = controller.update(
                    primary_target, {}, absolute_mode=is_absolute
                )

                # 7. Draw HUD
                current_cmds = (lr, fb, ud, yaw)
                draw_hud(
                    control_frame,
                    battery_level,
                    is_flying,
                    current_cmds,
                    primary_target,
                    controller,
                )

                # --- Drone HUD + Drone ArUco detection (no deadzone on drone view) ---
                drone_display = None
                drone_primary_target = None

                if drone_frame is not None:
                    # Flip only the camera image first
                    drone_display = cv2.flip(drone_frame, 1)

                    # Detect ArUco using the DRONE image (not webcam)
                    if config.VISION_MODE == "aruco":
                        drone_targets = vision.process(drone_display)

                        for t in drone_targets:
                            if t.id == active_target_id:
                                drone_primary_target = t
                                break

                        # Extra explicit message (optional)
                        if drone_primary_target is not None:
                            tx, ty, tz = drone_primary_target.tvec  # meters
                            ty = -ty  # opencv's positive y is downward

                            last_tx, last_ty, last_tz = tx, ty, tz
                            # Distance to marker
                            dist = np.linalg.norm([tx, ty, tz])
                            last_dist = dist
                    
                            found_frames += 1
                            lost_frames = 0
 
                            vec_text_1 = (
                                f"tvec (m): x={tx:+.2f}, y={ty:+.2f}, z={tz:+.2f}"
                            )
                            vec_text_2 = f"distance: {dist:.2f} m"

                            cv2.putText(
                                drone_display,
                                vec_text_1,
                                (30, 170),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 255, 0),
                                2,
                            )

                            cv2.putText(
                                drone_display,
                                vec_text_2,
                                (30, 200),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.7,
                                (0, 255, 255),
                                2,
                            )
                            cv2.putText(
                                drone_display,
                                "DRONE ARUCO DETECTED",
                                (30, 110),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 255, 0),
                                2,
                            )
                        else:
                            found_frames = 0
                            lost_frames += 1   
                            
                        if search_status:
                            cv2.putText(
                                drone_display,
                                search_status,
                                (600, 150),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.9,
                                (0, 255, 255),
                                2,
                            )
                        if search_idx != 0:
                            how_many_search = f"Search #{search_idx}"
                            cv2.putText(
                                drone_display,
                                how_many_search,
                                (600, 110),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 255, 0),
                                2,
                            )

                        # Draw HUD onto the flipped drone image, but NO deadzone
                        draw_hud(
                            drone_display,
                            battery_level,
                            is_flying,
                            current_cmds,
                            drone_primary_target,
                            controller,
                            show_deadzone=False,
                        )

                    cv2.imshow("Drone Stream", drone_display)
                    window_shown = True

            if control_frame is not None:
                cv2.imshow("Control View (Webcam)", control_frame)
                window_shown = True

            # --- Input ---
            # Ensure at least one window exists for waitKey to work!
            if not window_shown:
                # Create a black dummy window for status/input
                dummy = 255 * np.ones((400, 600, 3), dtype=np.uint8)
                draw_hud(dummy, battery_level, is_flying, controller=controller)
                cv2.putText(
                    dummy,
                    "NO VIDEO",
                    (200, 200),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )
                cv2.imshow("Drone Control Panel", dummy)

            key = cv2.waitKey(1) & 0xFF

            # Global Keys
            if key == ord("q"):
                break
            elif key == ord("t") or key == ord("T"):
                if not is_flying:
                    manual_input = True
                    # Safety Check: Calibration
                    is_calibrated = getattr(controller, "is_calibrated", True)
                    if config.VISION_MODE == "aruco" and not is_calibrated:
                        print("BLOCKED: Calibrate first!")  # Console log
                        pass
                    else:
                        drone.takeoff()
                        is_flying = True
            elif key == ord("l") or key == ord("L"):
                if is_flying:
                    manual_input = True
                    drone.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.05)
                    drone.land()
                    is_flying = False
            elif key == ord("c") or key == ord("C"):
                manual_input = True
                # Toggle search mode (only meaningful in aruco + flying)
                if config.VISION_MODE == "aruco" and is_flying:
                    search_enabled = not search_enabled
                    search_phase = "ROTATE"
                    search_phase_t0 = time.time()
                    # stop_yaw_frames = 0
                    search_status = (
                        "SEARCH ENABLED" if search_enabled else "SEARCH DISABLED"
                    )
                else:
                    search_status = "SEARCH: must be flying in ARUCO mode"

            # Manual Control Override (Emergency)
            # Hold 'm' to enable manual control (stops vision control)
            # This is simple; for toggle, we'd need a state var.
            # Let's use specific keys that OVERRIDE calculated logic for this frame.

            manual_input = False
            man_lr, man_fb, man_ud, man_yaw = 0, 0, 0, 0

            # WASD for Movement (Right Hand Mode 2 equivalent-ish)
            if key == ord("w"):  # Forward
                man_fb = 50
                manual_input = True
            elif key == ord("s"):  # Backward
                man_fb = -50
                manual_input = True
            elif key == ord("a"):  # Left
                man_lr = -50
                manual_input = True
            elif key == ord("d"):  # Right
                man_lr = 50
                manual_input = True

            # Arrow Keys (Up/Down/Yaw) - Map to I/K/J/U for easier keyboard access
            elif key == ord("i"):  # Up
                man_ud = 50
                manual_input = True
            elif key == ord("k"):  # Down
                man_ud = -50
                manual_input = True
            elif key == ord("j"):  # Yaw Left
                man_yaw = -50
                manual_input = True
            elif key == ord("u"):  # Yaw Right
                man_yaw = 50
                manual_input = True
                pass

            if manual_input:
                # Override Vision Commands
                lr, fb, ud, yaw = man_lr, man_fb, man_ud, man_yaw
                if control_frame is not None:
                    cv2.putText(
                        control_frame,
                        "MANUAL OVERRIDE",
                        (300, 100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 0, 255),
                        3,
                    )

            # --- Search yaw on DRONE camera when no ArUco is detected ---
            if (
                config.VISION_MODE == "aruco"
                and is_flying
                and (not manual_input)
                and search_enabled
            ):
                have_plan = (len(waypoints) > 0)
                mid_execution = have_plan and (wp_idx < numOfWP)
                now = time.time()
                rotate_time = SEARCH_STEP_DEG / float(SEARCH_YAW_DPS)
                if search_status == "MARKER FOUND" or search_status == "APPROACHING":

                    # if we are executing and the marker is lost -> enter search, do NOT reset wp_idx/waypoints
                    if drone_primary_target is None:
                        if lost_frames < LOST_FRAMES_TO_SEARCH:
                            search_status = "APPROACHING"
                            waypoint = waypoints[wp_idx]
                            lr, fb, ud, yaw = rc_to_reach_tvec(last_tx, last_ty, last_tz, waypoint)
                        else:
                            search_status = "YAWING (ROTATE)"
                            search_phase = "ROTATE"
                            search_phase_t0 = now
                            lr, fb, ud, yaw = 0, 0, 0, SEARCH_YAW_DPS

                    else:
                        lost_frames = 0
                        # marker visible: execute waypoint follower
                        if search_status == "MARKER FOUND":
                            search_status = "APPROACHING"
                        waypoint = waypoints[wp_idx]
                        if reached_waypoint(tx, ty, tz, waypoint):  
                            wp_idx += 1
                            if wp_idx == numOfWP:
                                wp_idx = 0
                                search_status = "ARRIVED"
                                lr, fb, ud, yaw = 0, 0, 0, 0
                            else:
                                waypoint = waypoints[wp_idx]
                                lr, fb, ud, yaw = rc_to_reach_tvec(tx, ty, tz, waypoint)                           
                        else:
                            lr, fb, ud, yaw = rc_to_reach_tvec(tx, ty, tz, waypoint)
                           
                        # stop condition
                        if abs(dist - 0.5) < 0.05:
                            wp_idx = 0
                            search_status = "ARRIVED"
                            lr, fb, ud, yaw = 0, 0, 0, 0

                elif drone_primary_target is None:
                    # normal search when we don't see marker
                    if search_phase == "ROTATE":
                        search_status = "YAWING (ROTATE)"
                        if (now - search_phase_t0) >= rotate_time:
                            search_phase = "PAUSE"
                            search_phase_t0 = now
                        lr, fb, ud, yaw = 0, 0, 0, SEARCH_YAW_DPS
                    else:
                        search_status = "SEARCH PAUSE"
                        if (now - search_phase_t0) >= SEARCH_PAUSE_S:
                            search_phase = "ROTATE"
                            search_phase_t0 = now
                            lr, fb, ud, yaw = 0, 0, 0, 0

                else:
                    # marker is visible here                  
                    lost_frames = 0
                    if search_status == "ARRIVED":
                        # waypoints = generate_spiral_waypoints(tx, ty, tz)
                        # wp_idx = 0
                        # search_idx += 1
                        # search_status = "MARKER FOUND"
                        lr, fb, ud, yaw = 0, 0, 0, 0

                    elif mid_execution:
                        # we were executing, marker was lost, now it's back -> resume SAME traj
                        search_status = "APPROACHING"
                        lr, fb, ud, yaw = 0, 0, 0, 0

                    else:
                        # no plan yet (or somehow lost it) -> plan once
                        waypoints = generate_spiral_waypoints(tx, ty, tz)
                        wp_idx = 0
                        search_idx += 1
                        search_status = "MARKER FOUND"
                        lr, fb, ud, yaw = 0, 0, 0, 0

            try:
                logger.info(
                    # f"LR:{lr:.2f} FB:{fb:.2f} UD:{ud:.2f} YAW:{yaw:.2f} IDX:{wp_idx} | STATUS:{search_status}"
                    f"lost_frame: {lost_frames}  IDX : {wp_idx}  waypoint: {waypoint} LR:{lr:.2f} FB:{fb:.2f} UD:{ud:.2f}"
                )
            except Exception:
                pass

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
