#!/usr/bin/env python3
import time
from aero_hand_lite.aero_hand import AeroHand

if __name__ == "__main__":
    hand = AeroHand("COM12")  # Update this to your serial port (e.g., "/dev/ttyUSB0" on Linux or "COM3" on Windows)
    # Example: trim channel 5 by +50 degrees
    hand.send_trim_mode(channel=5, degrees=50)
    print("Sent trim: channel=5, +50°")
