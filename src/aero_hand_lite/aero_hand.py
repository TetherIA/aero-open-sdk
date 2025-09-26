#!/usr/bin/env python3
import serial
import struct
import time
from aero_hand_lite.joints_to_actuations import JointsToActuationsModel

## Command Identifiers
CTRL_POS = 0x01

## Get Identifiers
GET_POS = 0x11
GET_VEL = 0x12
GET_CURR = 0x13
GET_TEMP = 0x14

## Rx indentifiers
RX_POS = 0x21
RX_VEL = 0x22
RX_CURR = 0x23
RX_TEMP = 0x24

JOINT_NAMES = [
    "thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip",
    "index_mcp_flex", "index_pip", "index_dip",
    "middle_mcp_flex", "middle_pip", "middle_dip",
    "ring_mcp_flex", "ring_pip", "ring_dip",
    "pinky_mcp_flex", "pinky_pip", "pinky_dip",
]

JOINT_LOWER_LIMITS = [0.0] * 16
JOINT_UPPER_LIMITS = [100.0, 55.0, 90.0, 90.0] + [90.0] * 12

ACTUATIONS_LOWER_LIMITS = [0.0, 0.0, -27.7778, 0.0, 0.0, 0.0, 0.0]
ACTUATIONS_UPPER_LIMITS = [100.0, 131.8906, 274.9275, 288.1603, 288.1603, 288.1603, 288.1603]

class AeroHand:
    def __init__(self, port = None, baudrate=115200):
        ## Connect to the serial port
        if port is None:
            self.ser = None
        else:
            self.ser = serial.Serial(port, baudrate, timeout=0.01, write_timeout=0.01)

        self.joint_names = JOINT_NAMES
        self.joint_lower_limits = JOINT_LOWER_LIMITS
        self.joint_upper_limits = JOINT_UPPER_LIMITS

        self.joints_to_actuations_model = JointsToActuationsModel()

    def set_joint_positions(self, positions: list):
        """
        Set the joint positions of the Aero Hand.

        Args:
            positions (list): A list of 16 joint positions. (degrees)
        """
        assert len(positions) == 16, "Expected 16 Joint Positions"

        ## Check for joint limits and raise a warning if out of bounds.
        assert all(
            self.joint_lower_limits[i] <= positions[i] <= self.joint_upper_limits[i]
            for i in range(16)
        ), "Warning: joint positions are out of bounds. Clamping to limits."

        ## Clamp the positions to the joint limits.
        positions = [
            max(self.joint_lower_limits[i], min(positions[i], self.joint_upper_limits[i]))
            for i in range(16)
        ]

        ## Convert to actuations
        actuations = self.joints_to_actuations_model.hand_actuations(positions)

        ## Normalize actuation to uint16 range. (0-65535)
        actuations = [
            (actuations[i] - ACTUATIONS_LOWER_LIMITS[i]) / (ACTUATIONS_UPPER_LIMITS[i] - ACTUATIONS_LOWER_LIMITS[i]) * 65535
            for i in range(7)
        ]

        ## Convert to Bytes
        msg = struct.pack('<2B7H', CTRL_POS, 0x00, *[int(a) for a in actuations])

        ## Send to serial
        if self.ser is not None:
            self.ser.write(msg)
            self.ser.flush()
    
    def get_forward_kinematics(self):
        raise NotImplementedError("This method is not yet implemented")

    def get_joint_positions(self):
        raise NotImplementedError("This method is not yet implemented")
    
    def get_motor_positions(self):
        ## Request joint positions from the hand
        msg = struct.pack('<2B', RX_POS, 0x00)
        self.ser.write(msg)
        self.ser.flush()

        ## Shoudl add delay or not?

        ## Read the response
        ## TODO: @mohitydv09 Read about read funtions
        resp = self.ser.read(2 + 7 * 2)  # 2 bytes header + 7 actuators * 2 bytes each
        data = struct.unpack('<2B7H', resp)
        if data[0] != RX_POS:
            raise ValueError("Invalid response from hand")
        return data

    def get_motor_currents(self):
        msg = struct.pack('<2B', RX_CURR, 0x00)
        self.ser.write(msg)
        self.ser.flush()

        ## Read the response
        start_time = time.perf_counter()
        resp = self.ser.read(2 + 7 * 2)  # 2
        end_time = time.perf_counter()
        print(f"Time taken to read currents: {end_time - start_time:.6f} seconds")
        data = struct.unpack('<2B7H', resp)
        if data[0] != RX_CURR:
            raise ValueError("Invalid response from hand")
        
        return data

    def get_motor_temperatures(self):
        msg = struct.pack('<2B', RX_TEMP, 0x00)
        self.ser.write(msg)
        self.ser.flush()

        ## Read the response
        start_time = time.perf_counter()
        resp = self.ser.read(2 + 7 * 2)  # 2
        end_time = time.perf_counter()
        print(f"Time taken to read temperatures: {end_time - start_time:.6f} seconds")
        print("Raw Response:", resp)
        data = struct.unpack('<2B7H', resp)
        if data[0] != RX_TEMP:
            raise ValueError("Invalid response from hand")
        return data

    def get_motor_speed(self):
        raise NotImplementedError("This method is not yet implemented")

    def close(self):
        self.ser.close()