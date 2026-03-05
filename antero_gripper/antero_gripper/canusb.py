import time
import serial

# CANUSB protocol constants
PACKET_START = 0xAA
CMD_START = 0x55
CMD_ID = 0x12
MODE_NORMAL = 0x00
FRAME_STANDARD = 0x01
FIXED_CAN_ID = 0x10
CAN_BITRATE = 250000
CAN_SPEED_CODES = {CAN_BITRATE: 0x05}


class CANUSB:
    """Serial-based CAN adapter driver for CH340 USB-to-CAN transceiver."""

    def __init__(self, port="/dev/ttyUSB0", baudrate=2000000):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0)
        time.sleep(0.1)
        self._configure_bus()

    def _configure_bus(self):
        cmd = bytearray(20)
        cmd[0], cmd[1], cmd[2] = PACKET_START, CMD_START, CMD_ID
        cmd[3], cmd[4] = CAN_SPEED_CODES[CAN_BITRATE], FRAME_STANDARD
        for i in range(5, 13):
            cmd[i] = 0x00
        cmd[14], cmd[15] = MODE_NORMAL, 0x01
        cmd[19] = sum(cmd[2:19]) & 0xFF
        self.ser.write(cmd)

    def send_frame(self, data_bytes):
        dlc = len(data_bytes)
        if not 0 <= dlc <= 8:
            raise ValueError("Data length must be 0-8 bytes")
        frame = bytearray([PACKET_START])
        info = (0xC0 & 0xEF) | dlc
        frame += bytearray([info, FIXED_CAN_ID & 0xFF, (FIXED_CAN_ID >> 8) & 0xFF])
        frame.extend(bytearray(data_bytes))
        frame.append(0x55)
        self.ser.write(frame)

    def close(self):
        self.ser.close()
