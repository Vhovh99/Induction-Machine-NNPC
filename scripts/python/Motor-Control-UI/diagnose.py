#!/usr/bin/env python3
"""
Motor Control UI - Serial Communication Diagnostic Tool

Tests binary protocol communication with the motor controller.
Run this before running the full UI application.
"""

import serial
import serial.tools.list_ports
import struct
import time
import sys


SYNC_BYTE = 0xAA
CMD_GET_STATUS = 0x07
CMD_MOTOR_START = 0x01
CMD_MOTOR_STOP = 0x02
RSP_ACK = 0x80
RSP_NACK = 0x81
RSP_STATUS = 0x83
RSP_TELEMETRY = 0x82

MOTOR_STATES = {0: "IDLE", 1: "FLUX_BUILD", 2: "RUNNING"}


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


def read_frame(ser, timeout=2.0):
    """Read a single binary frame from serial. Returns (cmd, payload) or None."""
    start = time.time()
    state = 0  # 0=SYNC, 1=CMD, 2=LEN, 3=PAYLOAD, 4=CRC
    cmd = 0
    plen = 0
    payload = bytearray()

    while time.time() - start < timeout:
        if ser.in_waiting == 0:
            time.sleep(0.001)
            continue
        b = ser.read(1)[0]

        if state == 0:
            if b == SYNC_BYTE:
                state = 1
        elif state == 1:
            cmd = b
            state = 2
        elif state == 2:
            if b > 56:
                state = 0
            else:
                plen = b
                payload = bytearray()
                state = 4 if plen == 0 else 3
        elif state == 3:
            payload.append(b)
            if len(payload) >= plen:
                state = 4
        elif state == 4:
            header = bytes([cmd, plen]) + bytes(payload)
            expected = crc8_maxim(header)
            if b == expected:
                return (cmd, bytes(payload))
            state = 0
    return None


def list_ports():
    print("\n" + "=" * 60)
    print("Available Serial Ports:")
    print("=" * 60)
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found!")
        return []
    for i, p in enumerate(ports, 1):
        print(f"{i}. {p.device}: {p.description}")
    return ports


def test_port(port_path):
    print(f"\n" + "=" * 60)
    print(f"Testing Port: {port_path} (binary protocol @ 230400)")
    print("=" * 60)

    try:
        ser = serial.Serial(port_path, baudrate=230400, timeout=1.0,
                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE)
        print(f"Connected to {port_path}")

        # Test 1: GET_STATUS
        print("\n[Test 1] Sending GET_STATUS...")
        ser.write(build_frame(CMD_GET_STATUS))
        result = read_frame(ser, timeout=2.0)
        if result:
            cmd, payload = result
            if cmd == RSP_STATUS and len(payload) == 12:
                state = payload[0]
                omega_ref = struct.unpack_from('<f', payload, 1)[0]
                id_ref = struct.unpack_from('<f', payload, 5)[0]
                faults = payload[9]
                uptime = struct.unpack_from('<H', payload, 10)[0]
                print(f"  STATUS: state={MOTOR_STATES.get(state, state)}, "
                      f"omega_ref={omega_ref:.2f}, id_ref={id_ref:.3f}, "
                      f"faults=0x{faults:02X}, uptime={uptime}s")
            else:
                print(f"  Got response cmd=0x{cmd:02X}, len={len(payload)}")
        else:
            print("  No response to GET_STATUS")

        # Test 2: Listen for telemetry
        print("\n[Test 2] Listening for telemetry (3 seconds)...")
        telem_count = 0
        start = time.time()
        while time.time() - start < 3:
            result = read_frame(ser, timeout=0.5)
            if result:
                cmd, payload = result
                if cmd == RSP_TELEMETRY and len(payload) == 28:
                    vals = struct.unpack('<fffffff', payload)
                    telem_count += 1
                    if telem_count <= 3:
                        print(f"  TELEM #{telem_count}: id={vals[0]:.3f} iq={vals[1]:.3f} "
                              f"vbus={vals[2]:.2f} omega={vals[3]:.2f} "
                              f"ia={vals[4]:.3f} ib={vals[5]:.3f} ic={vals[6]:.3f}")
        print(f"  Received {telem_count} telemetry packets in 3s")

        ser.close()
        print(f"\nClosed connection to {port_path}")
        return telem_count > 0

    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False


def main():
    print("\n" + "=" * 60)
    print("Motor Control UI - Binary Protocol Diagnostic")
    print("=" * 60)

    ports = list_ports()
    if not ports:
        print("\nNo serial ports found. Check hardware connection.")
        sys.exit(1)

    if len(ports) == 1:
        port = ports[0]
    else:
        print("\n" + "=" * 60)
        try:
            choice = int(input("Select port number to test (or 0 to exit): "))
            if choice == 0:
                sys.exit(0)
            if choice < 1 or choice > len(ports):
                print("Invalid selection!")
                sys.exit(1)
            port = ports[choice - 1]
        except ValueError:
            print("Invalid input!")
            sys.exit(1)

    success = test_port(port.device)

    print("\n" + "=" * 60)
    print("Diagnostic Summary:")
    print("=" * 60)

    if success:
        print("Motor controller is responding correctly")
        print("\nNext steps:")
        print("1. Run: python main.py")
        print(f"2. Select port: {port.device}")
        print("3. Click 'Connect'")
    else:
        print("Motor controller not responding")
        print("\nPossible issues:")
        print("1. Motor controller not powered on")
        print("2. Wrong serial port selected")
        print("3. Baudrate mismatch (expected: 230400)")
        print("4. USB/serial cable disconnected")
        print("5. Motor controller firmware issue")
        print("4. Verify motor controller supports 115200 baud")
    
    print("="*60 + "\n")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
