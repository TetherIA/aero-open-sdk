#!/usr/bin/env python3
import serial
import struct
import time
import math
from joints_to_actuations import JointsToActuationsModel

#Setup Modes
SUB_HOMING = 0x01
SUB_ZERO   = 0x02
SUB_SET_ID = 0x03
SUB_TRIM   = 0x04

## Command Identifiers
CTRL_POS = 0x11

## Get Identifiers
GET_ALL = 0x21
GET_POS = 0x22
GET_VEL = 0x23
GET_CURR = 0x24
GET_TEMP = 0x25

# ## Rx indentifiers
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
    def __init__(self, port = None, baudrate=921600):
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

        # ---------- NEW control helpers ----------
    # ---------- 16-byte command frames ----------
    def _send16(self, op: int, payload14: bytes = b''):
        if not self.ser:
            return
        if len(payload14) > 14:
            raise ValueError("payload must be <= 14 bytes")
        payload14 = payload14.ljust(14, b'\x00')
        frame = struct.pack('<BB', op & 0xFF, 0x00) + payload14  # 16 bytes total
        self.ser.write(frame); self.ser.flush()

    def send_homing(self):
        """[0x01, 0x00, 14×0]"""
        self._send16(0x01, b'')

    def send_zero(self):
        """[0x02, 0x00, 14×0]"""
        self._send16(0x02, b'')

    def send_set_id_mode(self, channel: int, new_id: int, current_limit_ma: int):
        """
        Payload layout (little-endian, total 14 bytes used):
            byte 0: channel (u8, 0..14)
            byte 1: new_id (u8, 0..250)
            byte 2..3: current_limit_ma (u16)
            bytes 4..13: reserved = 0
        """
        if not (0 <= channel <= 14): raise ValueError("channel must be 0..14")
        if not (0 <= new_id <= 250): raise ValueError("new_id must be 0..250")
        if not (0 <= current_limit_ma <= 5000): raise ValueError("current_limit_ma out of range")
        payload = struct.pack('<BBH', channel & 0xFF, new_id & 0xFF, current_limit_ma & 0xFFFF)
        self._send16(0x03, payload)

    def send_trim_mode(self, channel: int, delta_counts: int):
        """
        Payload layout:
            byte 0: channel (u8, 0..14)
            byte 1..2: delta_counts (i16, e.g. -4095..+4095)
            bytes 3..13: reserved = 0
        """
        if not (0 <= channel <= 14): raise ValueError("channel must be 0..14")
        if not (-4095 <= delta_counts <= 4095): raise ValueError("delta_counts out of range")
        payload = struct.pack('<Bh', channel & 0xFF, int(delta_counts))
        self._send16(0x04, payload)

    def send_actuations_raw(self, u16_vals):
        """
        Send 7 raw actuator commands (0..65535) directly in CTRL_POS frame.
        Order must match firmware's SERVO_IDS order.
        """
        assert len(u16_vals) == 7
        clamped = [max(0, min(int(v), 65535)) for v in u16_vals]
        frame = struct.pack('<2B7H', CTRL_POS, 0x00, *clamped)
        self.ser.write(frame); self.ser.flush()

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
        #print("Raw Response:", resp)
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
    

if __name__ == "__main__":
    hand = AeroHand("COM12")
    # Should open (extend)
    #hand.send_homing()
    #time.sleep(60)
    # Should also open (same as above)
    #hand.send_actuations_raw([0]*7)
    #time.sleep(5)
    # hand.set_joint_positions([10.0, 20.0, 30.0, 40.0] + [45.0] * 12)
    while True:
        try:
            #hand.send_zero()
            #time.sleep(5)
            # hand.send_actuations_raw([0]*7)
            # time.sleep(5)
            # hand.send_actuations_raw([10000]*7)
            # time.sleep(5)
            # hand.send_actuations_raw([20000]*7)
            # time.sleep(5)

            # Unique values per actuator so you can tell who is who
            #hand.send_actuations_raw([6000, 6000, 5000, 65000, 65000, 65000, 65000])
            #hand.get_motor_temperatures()
            step = 0
            while True:
                # First four joints vary slowly with sine waves
                j0 = 30.0 + 15.0 * (1 + math.sin(step * 0.05))    # oscillates 10–20
                j1 = 10.0 + 5.0 * (1 + math.sin(step * 0.07))    # oscillates 20–30
                j2 = 10.0 + 5.0 * (1 + math.sin(step * 0.09))    # oscillates 30–40
                j3 = 10.0 + 5.0 * (1 + math.sin(step * 0.11))    # oscillates 40–50

                # Remaining 12 joints hover around 50 with small offsets
                rest = [30.0 + 20.0 * math.sin(0.05 * step + i) for i in range(12)]

                # Send positions to the hand
                hand.set_joint_positions([j0, j1, j2, j3] + rest)

                # Increment and sleep
                step += 1
                time.sleep(0.02)  # ~50 Hz update
        except Exception as e:
            print("Error:", e)
            continue
    # temps = hand.get_motor_currents()
    # hand.close()

    # print("Curent:", temps)