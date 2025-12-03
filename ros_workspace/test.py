# Save as test_djitellopy.py
from djitellopy import Tello
import time

print("Testing Tello connection...")
tello = Tello()

# Enable verbose logging
import logging
logging.basicConfig(level=logging.DEBUG)

try:
    print("Attempting to connect...")
    tello.connect(wait_for_state=False)  # Skip state check
    print("✓ Command connection successful!")
    
    print("Getting battery...")
    battery = tello.get_battery()
    print(f"✓ Battery: {battery}%")
    
    print("Getting temperature...")
    temp = tello.get_temperature()
    print(f"✓ Temperature: {temp}°C")

    print("Will take off in 3 seconds")
    time.sleep(3)
    tello.takeoff()
    
except Exception as e:
    print(f"✗ Error: {e}")
