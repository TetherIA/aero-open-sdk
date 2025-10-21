#!/usr/bin/env python3
import time
from aero_open_sdk.aero_hand import AeroHand

if __name__ == "__main__":
    hand = AeroHand(
        "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_D8:3B:DA:45:C8:1C-if00"
    )

    # Note : This example demonstrates torque control for a robotic hand where you can try to grasp objects by applying varying torque values to the actuators 
    # or you can simply see the effect of torque control on the hand's movement by doing a handshake with the robotic hand.
    # Adjust the time.sleep() values to change the speed of torque variation and hold and release durations.
    # Sweep torque value for all 7 actuators from 0 to 1000 and back, then pause at each end
    torque = 0
    dir = 1
    while True:
        torque_list = [torque] * 7
        hand.ctrl_torque(torque_list)
        if dir == 1:
            torque += 1
            if torque >= 1000:
                dir = -1
                time.sleep(3)  # Pause at max torque - Handshake Position
        else:
            torque -= 1
            if torque <= 0:
                dir = 1
                time.sleep(3)  # Release Torque for 3 seconds - Handshake Release
        time.sleep(0.002)  # 0.002*1000 = 2 seconds for full sweep