from aero_hand_lite.aero_hand import AeroHand


if __name__ == "__main__":
    hand = AeroHand("/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_D8:3B:DA:45:C8:1C-if00")

    # hand.set_joint_positions([10.0, 20.0, 30.0, 40.0] + [45.0] * 12)
    while True:
        try:
            hand.get_motor_currents()
        except Exception as e:
            print("Error:", e)
            continue
    # temps = hand.get_motor_currents()
    # hand.close()

    # print("Curent:", temps)