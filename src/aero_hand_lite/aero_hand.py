#!/usr/bin/env python3
import serial
import struct

from aero_hand_lite.joints_to_actuations import JointsToActuationsModel

## Setup Modes
HOMING_MODE = 0x01
ZERO_MODE = 0x02
SET_ID_MODE = 0x03
TRIM_MODE = 0x04

## Command Modes
CTRL_POS = 0x11

## Request Modes
GET_ALL = 0x21
GET_POS = 0x22
GET_VEL = 0x23
GET_CURR = 0x24
GET_TEMP = 0x25


## Robot Constants
_JOINT_NAMES = [
    "thumb_cmc_abd",
    "thumb_cmc_flex",
    "thumb_mcp",
    "thumb_ip",
    "index_mcp_flex",
    "index_pip",
    "index_dip",
    "middle_mcp_flex",
    "middle_pip",
    "middle_dip",
    "ring_mcp_flex",
    "ring_pip",
    "ring_dip",
    "pinky_mcp_flex",
    "pinky_pip",
    "pinky_dip",
]

_JOINT_LOWER_LIMITS = [0.0] * 16
_JOINT_UPPER_LIMITS = [100.0, 55.0, 90.0, 90.0] + [90.0] * 12

_ACTUATIONS_LOWER_LIMITS = [0.0, 0.0, -27.7778, 0.0, 0.0, 0.0, 0.0]
_ACTUATIONS_UPPER_LIMITS = [
    100.0,
    131.8906,
    274.9275,
    288.1603,
    288.1603,
    288.1603,
    288.1603,
]

_UINT16_MAX = 65535


class AeroHand:
    def __init__(self, port=None, baudrate=921600):
        ## Connect to serial port
        if port is None:
            ## Lazy initialization for testing without hardware
            self.ser = None
        else:
            self.ser = serial.Serial(port, baudrate, timeout=0.01, write_timeout=0.01)

        self.joint_names = _JOINT_NAMES
        self.joint_lower_limits = _JOINT_LOWER_LIMITS
        self.joint_upper_limits = _JOINT_UPPER_LIMITS

        self.joints_to_actuations_model = JointsToActuationsModel()

    def set_joint_positions(self, positions: list):
        """
        Set the joint positions of the Aero Hand.

        Args:
            positions (list): A list of 16 joint positions. (degrees)
        """
        assert len(positions) == 16, "Expected 16 Joint Positions"

        ## Clamp the positions to the joint limits.
        positions = [
            max(
                self.joint_lower_limits[i],
                min(positions[i], self.joint_upper_limits[i]),
            )
            for i in range(16)
        ]

        ## Convert to actuations
        actuations = self.joints_to_actuations_model.hand_actuations(positions)

        ## Normalize actuation to uint16 range. (0-65535)
        actuations = [
            (actuations[i] - _ACTUATIONS_LOWER_LIMITS[i])
            / (_ACTUATIONS_UPPER_LIMITS[i] - _ACTUATIONS_LOWER_LIMITS[i])
            * _UINT16_MAX
            for i in range(7)
        ]

        self._send_data(CTRL_POS, [int(a) for a in actuations])

    def _send_data(self, header: int, payload: list[int] = [0] * 7):
        assert self.ser is not None, "Serial port is not initialized"
        assert len(payload) == 7, "Payload must be a list of 7 integers"
        msg = struct.pack("<2B7H", header & 0xFF, 0x00, *(v & 0xFFFF for v in payload))
        self.ser.write(msg)
        self.ser.flush()

    def send_homing(self):
        self._send_data(HOMING_MODE)

    def send_zero(self):
        self._send_data(ZERO_MODE)

    def get_forward_kinematics(self):
        raise NotImplementedError("This method is not yet implemented")

    def get_joint_positions(self):
        raise NotImplementedError("This method is not yet implemented")

    def get_motor_positions(self):
        """
        Get the motor positions from the hand.
        Returns:
            list: A list of 7 motor positions. (degrees)
        """
        return self._get_info(GET_POS)

    def get_motor_currents(self):
        """
        Get the motor currents from the hand.
        Returns:
            list: A list of 7 motor currents. (mA)
        """
        return self._get_info(GET_CURR)

    def get_motor_temperatures(self):
        """
        Get the motor temperatures from the hand.
        Returns:
            list: A list of 7 motor temperatures. (Degree Celsius)
        """
        return self._get_info(GET_TEMP)

    def get_motor_speed(self):
        """
        Get the motor speeds from the hand.
        Returns:
            list: A list of 7 motor speeds. (RPM)
        """
        return self._get_info(GET_VEL)

    def _get_info(self, code: int):
        """
        Get the info from the hand.
        Args:
            code (int): The info code to get.
        Returns:
            list: A list of 7 info values.
        """
        self._send_data(code)
        ## Read the response
        resp = self.ser.read(2 + 7 * 2)  # 2
        data = struct.unpack("<2B7H", resp)
        if data[0] != code:
            raise ValueError("Invalid response from hand")
        return data[2:]

    def close(self):
        self.ser.close()
