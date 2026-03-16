#!/usr/bin/env python3
"""
Mock Motor Controller Simulator — Binary Protocol

Simulates the firmware's binary serial protocol for testing the UI
without a physical motor controller.

Usage:
    python mock_controller.py /dev/ttyVCOM0
    # Then connect the UI to /dev/ttyVCOM1
"""

import serial
import struct
import time
import math
import argparse

# Protocol constants (mirror serial_communicator.py / firmware)
SYNC_BYTE = 0xAA
MAX_PAYLOAD = 56

CMD_MOTOR_START = 0x01
CMD_MOTOR_STOP = 0x02
CMD_SET_SPEED_REF = 0x03
CMD_SET_ID_REF = 0x04
CMD_SET_CURRENT_PI = 0x05
CMD_SET_SPEED_PI = 0x06
CMD_GET_STATUS = 0x07
CMD_SET_TELEMETRY_DIV = 0x08
CMD_SET_LOAD = 0x09

RSP_ACK = 0x80
RSP_NACK = 0x81
RSP_TELEMETRY = 0x82
RSP_STATUS = 0x83

ERR_UNKNOWN_CMD = 0x01
ERR_BAD_LENGTH = 0x02
ERR_BAD_CRC = 0x03

# RX states
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


def build_frame(cmd: int, payload: bytes = b'') -> bytes:
    header = bytes([cmd, len(payload)]) + payload
    crc = crc8_maxim(header)
    return bytes([SYNC_BYTE]) + header + bytes([crc])


class MockMotorController:
    """Simulates a motor controller using the binary serial protocol."""

    def __init__(self, port, baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False

        # Motor state
        self.motor_state = 0  # 0=IDLE, 1=FLUX_BUILD, 2=RUNNING
        self.omega_m = 0.0  # mechanical speed rad/s
        self.speed_ref = 0.0  # speed reference rad/s
        self.id_ref = 0.4  # d-axis current ref A
        self.tau = 0.5
        self.last_update = time.time()
        self.start_time = time.time()
        self.flux_build_start = 0.0

        # Telemetry
        self.telem_divider = 200
        self.telem_counter = 0
        self.phase = 0.0

        # RX state machine
        self._rx_state = _RX_SYNC
        self._rx_cmd = 0
        self._rx_len = 0
        self._rx_payload = bytearray()
        self._rx_idx = 0

    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=0.01,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self.start_time = time.time()
            print(f"Mock controller listening on {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"Failed to open {self.port}: {e}")
            return False

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def run(self):
        self.running = True
        while self.running:
            # Read incoming bytes
            if self.serial.in_waiting > 0:
                data = self.serial.read(self.serial.in_waiting)
                for b in data:
                    self._rx_feed(b)

            self._update_dynamics()

            # Telemetry tick
            if self.telem_divider > 0:
                self.telem_counter += 1
                if self.telem_counter >= self.telem_divider:
                    self.telem_counter = 0
                    self._send_telemetry()

            time.sleep(0.001)

    # ---- RX state machine (mirrors firmware) ----

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
            expected = crc8_maxim(header)
            if byte == expected:
                self._process_command(self._rx_cmd, bytes(self._rx_payload))
            else:
                print(f"  CRC mismatch for cmd 0x{self._rx_cmd:02X}")
            self._rx_state = _RX_SYNC

    def _send(self, frame: bytes):
        self.serial.write(frame)

    def _send_ack(self, cmd_echo: int):
        self._send(build_frame(RSP_ACK, bytes([cmd_echo])))

    def _send_nack(self, cmd_echo: int, error: int):
        self._send(build_frame(RSP_NACK, bytes([cmd_echo, error])))

    def _process_command(self, cmd: int, payload: bytes):
        plen = len(payload)

        if cmd == CMD_MOTOR_START:
            if plen != 0:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            self.motor_state = 1  # FLUX_BUILD
            self.flux_build_start = time.time()
            print("  MOTOR_START")
            self._send_ack(cmd)

        elif cmd == CMD_MOTOR_STOP:
            if plen != 0:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            self.motor_state = 0
            self.speed_ref = 0.0
            self.omega_m = 0.0
            print("  MOTOR_STOP")
            self._send_ack(cmd)

        elif cmd == CMD_SET_SPEED_REF:
            if plen != 4:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            self.speed_ref = struct.unpack('<f', payload)[0]
            print(f"  SET_SPEED_REF: {self.speed_ref:.2f} rad/s")
            self._send_ack(cmd)

        elif cmd == CMD_SET_ID_REF:
            if plen != 4:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            self.id_ref = struct.unpack('<f', payload)[0]
            print(f"  SET_ID_REF: {self.id_ref:.3f} A")
            self._send_ack(cmd)

        elif cmd == CMD_SET_CURRENT_PI:
            if plen != 8:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            kp, ki = struct.unpack('<ff', payload)
            print(f"  SET_CURRENT_PI: Kp={kp}, Ki={ki}")
            self._send_ack(cmd)

        elif cmd == CMD_SET_SPEED_PI:
            if plen != 8:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            kp, ki = struct.unpack('<ff', payload)
            print(f"  SET_SPEED_PI: Kp={kp}, Ki={ki}")
            self._send_ack(cmd)

        elif cmd == CMD_GET_STATUS:
            if plen != 0:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            uptime = int(time.time() - self.start_time) & 0xFFFF
            status_payload = struct.pack('<BfBH',
                self.motor_state, self.speed_ref,
                0, uptime)
            # Insert id_ref between omega_ref and fault_flags
            # Layout: u8 motor_state, f32 omega_ref, f32 id_ref, u8 fault_flags, u16 uptime_s
            status_payload = struct.pack('<BffBH',
                self.motor_state, self.speed_ref, self.id_ref, 0, uptime)
            self._send(build_frame(RSP_STATUS, status_payload))
            print(f"  GET_STATUS -> state={self.motor_state}")

        elif cmd == CMD_SET_TELEMETRY_DIV:
            if plen != 2:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            self.telem_divider = struct.unpack('<H', payload)[0]
            self.telem_counter = 0
            print(f"  SET_TELEMETRY_DIV: {self.telem_divider}")
            self._send_ack(cmd)

        elif cmd == CMD_SET_LOAD:
            if plen != 1:
                self._send_nack(cmd, ERR_BAD_LENGTH); return
            load_count = payload[0]
            print(f"  SET_LOAD: {load_count} relay(s)")
            self._send_ack(cmd)

        else:
            self._send_nack(cmd, ERR_UNKNOWN_CMD)
            print(f"  Unknown cmd: 0x{cmd:02X}")

    # ---- Motor dynamics simulation ----

    def _update_dynamics(self):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        if self.motor_state == 1:  # FLUX_BUILD
            if now - self.flux_build_start >= 0.2:
                self.motor_state = 2  # RUNNING
                print("  -> RUNNING")

        if self.motor_state == 2:
            error = self.speed_ref - self.omega_m
            self.omega_m += (error / self.tau) * dt
        elif self.motor_state == 0:
            self.omega_m *= (1.0 - 2.0 * dt)
            if abs(self.omega_m) < 0.01:
                self.omega_m = 0.0

        self.phase += abs(self.omega_m) * dt

    def _send_telemetry(self):
        # Simulate currents
        cur_mag = abs(self.omega_m) * 0.01 + self.id_ref
        id_val = self.id_ref * 0.95
        iq_val = (self.omega_m * 0.005) if self.motor_state == 2 else 0.0
        vbus = 24.0 + math.sin(time.time() * 0.1) * 0.3
        ia = cur_mag * math.sin(self.phase)
        ib = cur_mag * math.sin(self.phase + 2.094)
        ic = cur_mag * math.sin(self.phase + 4.189)
        theta_e = math.fmod(self.phase * 2.0, 2.0 * math.pi)
        torque_e = iq_val * 1.5  # simplified torque estimate (N·m)
        imr = self.id_ref * 0.9  # simplified magnetising current
        dwr_dt = (self.speed_ref - self.omega_m) / self.tau  # acceleration estimate

        payload = struct.pack('<fffffffffff', id_val, iq_val, vbus,
                              self.omega_m, ia, ib, ic, theta_e, torque_e, imr, dwr_dt)
        self._send(build_frame(RSP_TELEMETRY, payload))


def main():
    parser = argparse.ArgumentParser(description='Mock Motor Controller (Binary Protocol)')
    parser.add_argument('port', help='Serial port to listen on')
    parser.add_argument('--baudrate', type=int, default=230400)
    args = parser.parse_args()

    controller = MockMotorController(args.port, args.baudrate)
    if controller.connect():
        try:
            controller.run()
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            controller.disconnect()


if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
