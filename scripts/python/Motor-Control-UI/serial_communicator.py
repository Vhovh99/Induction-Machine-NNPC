import serial
import struct
import threading
import queue
import time
from typing import Optional
from config import SERIAL_BAUDRATE, SERIAL_TIMEOUT

# --- Protocol Constants (match firmware serial_protocol.h) ---
SYNC_BYTE = 0xAA
MAX_PAYLOAD = 56

# Command IDs (Host -> MCU)
CMD_MOTOR_START = 0x01
CMD_MOTOR_STOP = 0x02
CMD_SET_SPEED_REF = 0x03
CMD_SET_ID_REF = 0x04
CMD_SET_CURRENT_PI = 0x05
CMD_SET_SPEED_PI = 0x06
CMD_GET_STATUS = 0x07
CMD_SET_TELEMETRY_DIV = 0x08

# Response IDs (MCU -> Host)
RSP_ACK = 0x80
RSP_NACK = 0x81
RSP_TELEMETRY = 0x82
RSP_STATUS = 0x83

# NACK error codes
ERR_UNKNOWN_CMD = 0x01
ERR_BAD_LENGTH = 0x02
ERR_BAD_CRC = 0x03
ERR_INVALID_VALUE = 0x04

NACK_ERRORS = {
    ERR_UNKNOWN_CMD: "Unknown command",
    ERR_BAD_LENGTH: "Bad payload length",
    ERR_BAD_CRC: "CRC check failed",
    ERR_INVALID_VALUE: "Invalid value",
}

# Motor states
MOTOR_STATES = {0: "IDLE", 1: "FLUX_BUILD", 2: "RUNNING"}

# RX state machine states
_RX_SYNC = 0
_RX_CMD = 1
_RX_LEN = 2
_RX_PAYLOAD = 3
_RX_CRC = 4


def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class SerialCommunicator:
    """Handles binary serial communication with the motor controller."""

    def __init__(self, port: Optional[str] = None, baudrate: int = SERIAL_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = False
        self.receive_thread = None

        # RX state machine
        self._rx_state = _RX_SYNC
        self._rx_cmd = 0
        self._rx_len = 0
        self._rx_payload = bytearray()
        self._rx_idx = 0

        # Callbacks
        self.on_data_received = None  # telemetry dict
        self.on_status_received = None  # status dict
        self.on_ack_received = None  # cmd_echo int
        self.on_nack_received = None  # (cmd_echo, error_code)
        self.on_connection_change = None
        self.on_error = None

    def list_ports(self):
        import serial.tools.list_ports
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port: str) -> bool:
        try:
            self.port = port
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self._rx_state = _RX_SYNC
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()
            if self.on_connection_change:
                self.on_connection_change(True, f"Connected to {port}")
            return True
        except Exception as e:
            self.running = False
            if self.on_error:
                self.on_error(f"Connection error: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                if self.on_error:
                    self.on_error(f"Disconnection error: {e}")
        if self.on_connection_change:
            self.on_connection_change(False, "Disconnected")

    def is_connected(self) -> bool:
        return self.serial_port is not None and self.serial_port.is_open and self.running

    # ---- Frame building & sending ----

    def _build_frame(self, cmd: int, payload: bytes = b'') -> bytes:
        header = bytes([cmd, len(payload)]) + payload
        crc = crc8_maxim(header)
        return bytes([SYNC_BYTE]) + header + bytes([crc])

    def _send_frame(self, cmd: int, payload: bytes = b'') -> bool:
        if not self.is_connected():
            if self.on_error:
                self.on_error("Not connected to motor controller")
            return False
        try:
            frame = self._build_frame(cmd, payload)
            self.serial_port.write(frame)
            return True
        except Exception as e:
            if self.on_error:
                self.on_error(f"Send error: {e}")
            return False

    # ---- Public command methods ----

    def start_motor(self) -> bool:
        return self._send_frame(CMD_MOTOR_START)

    def stop_motor(self) -> bool:
        return self._send_frame(CMD_MOTOR_STOP)

    def set_speed_ref(self, omega_rad_s: float) -> bool:
        return self._send_frame(CMD_SET_SPEED_REF, struct.pack('<f', omega_rad_s))

    def set_id_ref(self, id_ref_a: float) -> bool:
        return self._send_frame(CMD_SET_ID_REF, struct.pack('<f', id_ref_a))

    def set_current_pi(self, kp: float, ki: float) -> bool:
        return self._send_frame(CMD_SET_CURRENT_PI, struct.pack('<ff', kp, ki))

    def set_speed_pi(self, kp: float, ki: float) -> bool:
        return self._send_frame(CMD_SET_SPEED_PI, struct.pack('<ff', kp, ki))

    def get_status(self) -> bool:
        return self._send_frame(CMD_GET_STATUS)

    def set_telemetry_div(self, divider: int) -> bool:
        divider = max(0, min(0xFFFF, int(divider)))
        return self._send_frame(CMD_SET_TELEMETRY_DIV, struct.pack('<H', divider))

    # ---- RX state machine ----

    def _receive_loop(self):
        while self.running:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for byte in data:
                        self._rx_feed(byte)
                else:
                    time.sleep(0.001)
            except Exception as e:
                if self.running:
                    if self.on_error:
                        self.on_error(f"Receive error: {e}")
                    self.running = False

    def _rx_feed(self, byte: int):
        if self._rx_state == _RX_SYNC:
            if byte == SYNC_BYTE:
                self._rx_state = _RX_CMD
        elif self._rx_state == _RX_CMD:
            self._rx_cmd = byte
            self._rx_state = _RX_LEN
        elif self._rx_state == _RX_LEN:
            if byte > MAX_PAYLOAD:
                self._rx_state = _RX_SYNC
            else:
                self._rx_len = byte
                self._rx_payload = bytearray()
                self._rx_idx = 0
                self._rx_state = _RX_CRC if byte == 0 else _RX_PAYLOAD
        elif self._rx_state == _RX_PAYLOAD:
            self._rx_payload.append(byte)
            self._rx_idx += 1
            if self._rx_idx >= self._rx_len:
                self._rx_state = _RX_CRC
        elif self._rx_state == _RX_CRC:
            header = bytes([self._rx_cmd, self._rx_len]) + bytes(self._rx_payload)
            expected_crc = crc8_maxim(header)
            if byte == expected_crc:
                self._process_response(self._rx_cmd, bytes(self._rx_payload))
            self._rx_state = _RX_SYNC

    def _process_response(self, cmd: int, payload: bytes):
        if cmd == RSP_ACK and len(payload) == 1:
            if self.on_ack_received:
                self.on_ack_received(payload[0])

        elif cmd == RSP_NACK and len(payload) == 2:
            if self.on_nack_received:
                self.on_nack_received(payload[0], payload[1])

        elif cmd == RSP_TELEMETRY and len(payload) == 36:
            # id(f), iq(f), vbus(f), omega_m(f/rad/s), ia(f), ib(f), ic(f), theta_e(f), torque_e(f)
            vals = struct.unpack('<fffffffff', payload)
            data = {
                'id': vals[0],
                'iq': vals[1],
                'vbus': vals[2],
                'omega_m': vals[3],
                'ia': vals[4],
                'ib': vals[5],
                'ic': vals[6],
                'theta_e': vals[7],
                'torque_e': vals[8],
                'timestamp': time.time(),
            }
            if self.on_data_received:
                self.on_data_received(data)

        elif cmd == RSP_STATUS and len(payload) == 12:
            # motor_state(u8), omega_ref(f), id_ref(f), fault_flags(u8), uptime_s(u16)
            motor_state = payload[0]
            omega_ref = struct.unpack_from('<f', payload, 1)[0]
            id_ref = struct.unpack_from('<f', payload, 5)[0]
            fault_flags = payload[9]
            uptime_s = struct.unpack_from('<H', payload, 10)[0]
            status = {
                'motor_state': motor_state,
                'motor_state_str': MOTOR_STATES.get(motor_state, f"UNKNOWN({motor_state})"),
                'omega_ref': omega_ref,
                'id_ref': id_ref,
                'fault_flags': fault_flags,
                'uptime_s': uptime_s,
            }
            if self.on_status_received:
                self.on_status_received(status)
