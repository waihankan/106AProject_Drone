# keyboard_tello.py
from djitellopy import Tello
from pynput import keyboard
import time
import threading

SPEED = 40  # cm/s per command
ROTATION_SPEED = 40  # deg/s

tello = Tello()
connected = False
flying = False

print("Connecting to Tello...")
tello.connect()
battery = tello.get_battery()
print(f"Connected! Battery: {battery}%")

# Optional: start video (comment out if not needed)
# tello.streamon()

# --- Movement flags ---
keys_pressed = {
    "forward": False,
    "back": False,
    "left": False,
    "right": False,
    "up": False,
    "down": False,
    "cw": False,
    "ccw": False
}

def send_commands():
    """ Continuously send movement commands while keys are held """
    while True:
        if flying:
            fb = lr = ud = yaw = 0

            if keys_pressed["forward"]: fb = SPEED
            if keys_pressed["back"]:    fb = -SPEED
            if keys_pressed["left"]:    lr = -SPEED
            if keys_pressed["right"]:   lr = SPEED
            if keys_pressed["up"]:      ud = SPEED
            if keys_pressed["down"]:    ud = -SPEED
            if keys_pressed["cw"]:      yaw = ROTATION_SPEED
            if keys_pressed["ccw"]:     yaw = -ROTATION_SPEED

            tello.send_rc_control(lr, fb, ud, yaw)

        time.sleep(0.05)  # 20 commands/sec


threading.Thread(target=send_commands, daemon=True).start()

# --- Keyboard control handler ---
def on_press(key):
    global flying

    try:
        if key.char == 'w': keys_pressed["forward"] = True
        if key.char == 's': keys_pressed["back"] = True
        if key.char == 'a': keys_pressed["left"] = True
        if key.char == 'd': keys_pressed["right"] = True

    except:
        pass

    # Arrow keys and special keys
    if key == keyboard.Key.up:    keys_pressed["up"] = True
    if key == keyboard.Key.down:  keys_pressed["down"] = True
    if key == keyboard.Key.left:  keys_pressed["ccw"] = True
    if key == keyboard.Key.right: keys_pressed["cw"] = True

    # Takeoff
    if key == keyboard.Key.space and not flying:
        print("Taking off...")
        tello.takeoff()
        flying = True

    # Land
    if key == keyboard.Key.ctrl_l and flying:
        print("Landing...")
        tello.land()
        flying = False

    # Emergency / exit
    if key == keyboard.Key.esc:
        print("EMERGENCY STOP!")
        try:
            tello.emergency()
        except:
            pass
        return False  # Stop listener


def on_release(key):
    try:
        if key.char == 'w': keys_pressed["forward"] = False
        if key.char == 's': keys_pressed["back"] = False
        if key.char == 'a': keys_pressed["left"] = False
        if key.char == 'd': keys_pressed["right"] = False
    except:
        pass

    if key == keyboard.Key.up:    keys_pressed["up"] = False
    if key == keyboard.Key.down:  keys_pressed["down"] = False
    if key == keyboard.Key.left:  keys_pressed["ccw"] = False
    if key == keyboard.Key.right: keys_pressed["cw"] = False


print("""
=============================
   KEYBOARD CONTROLS
=============================
W/S = forward/back
A/D = left/right
Arrow Up/Down = up/down
Arrow Left/Right = rotate CCW/CW

SPACE = takeoff
Left CTRL = land
ESC = emergency + exit
=============================
""")

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

print("Shutting down...")
tello.end()
