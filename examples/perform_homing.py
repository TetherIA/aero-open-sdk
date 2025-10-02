#!/usr/bin/env python3
from aero_hand_lite.aero_hand import AeroHand

if __name__ == "__main__":
    hand = AeroHand(
        "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_D8:3B:DA:45:CA:08-if00"
    )

    hand.send_homing()
    try:
        while True:
            line = hand.ser.readline().decode(errors="ignore").strip()
            if line:
                print(line)
    except KeyboardInterrupt:
        pass