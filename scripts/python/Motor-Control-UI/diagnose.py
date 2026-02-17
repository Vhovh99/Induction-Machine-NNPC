#!/usr/bin/env python3
"""
Motor Control UI - Serial Communication Diagnostic Tool

This script helps diagnose serial communication issues with the motor controller.
Run this before running the full UI application.
"""

import serial
import serial.tools.list_ports
import time
import sys


def list_ports():
    """List all available serial ports."""
    print("\n" + "="*60)
    print("Available Serial Ports:")
    print("="*60)
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found!")
        return []
    
    for i, p in enumerate(ports, 1):
        print(f"{i}. {p.device}: {p.description}")
    return ports


def test_port(port_path):
    """Test communication on a specific port."""
    print(f"\n" + "="*60)
    print(f"Testing Port: {port_path}")
    print("="*60)
    
    try:
        ser = serial.Serial(port_path, baudrate=115200, timeout=1.0)
        print(f"✓ Connected to {port_path}")
        
        # Test 1: Send a simple command
        print("\n[Test 1] Sending Frequency Command (F 50.0)...")
        ser.write(b"F 50.0\n")
        print("✓ Command sent")
        
        # Test 2: Wait for response
        print("\n[Test 2] Waiting for telemetry (5 seconds)...")
        received = False
        start_time = time.time()
        
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                line = ser.readline().decode('ascii').strip()
                if line:
                    print(f"✓ RX: {repr(line)}")
                    received = True
            time.sleep(0.1)
        
        if not received:
            print("✗ No telemetry received (expected: SPD:xxx CUR:x,y)")
        
        # Test 3: Send Start command
        print("\n[Test 3] Sending Start Command (S)...")
        ser.write(b"S\n")
        print("✓ Command sent")
        
        # Test 4: Wait for data
        print("[Test 4] Waiting for telemetry data (5 seconds)...")
        received = False
        start_time = time.time()
        
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                line = ser.readline().decode('ascii').strip()
                if line:
                    print(f"✓ RX: {repr(line)}")
                    received = True
            time.sleep(0.1)
        
        if not received:
            print("✗ No telemetry received after START")
        else:
            print("✓ Motor controller appears to be working!")
        
        # Test 5: Send Stop command
        print("\n[Test 5] Sending Stop Command (X)...")
        ser.write(b"X\n")
        print("✓ Stop command sent")
        
        ser.close()
        print(f"\n✓ Closed connection to {port_path}")
        
        return received
        
    except serial.SerialException as e:
        print(f"✗ Serial Error: {e}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def main():
    """Main diagnostic routine."""
    print("\n" + "="*60)
    print("Motor Control UI - Serial Communication Diagnostic")
    print("="*60)
    
    # List available ports
    ports = list_ports()
    
    if not ports:
        print("\n✗ No serial ports found. Check hardware connection.")
        sys.exit(1)
    
    if len(ports) == 1:
        port = ports[0]
        choice = 1
    else:
        # Multiple ports - ask user
        print("\n" + "="*60)
        try:
            choice = int(input("Select port number to test (or 0 to exit): "))
            if choice == 0:
                print("Exiting...")
                sys.exit(0)
            if choice < 1 or choice > len(ports):
                print("Invalid selection!")
                sys.exit(1)
            port = ports[choice - 1]
        except ValueError:
            print("Invalid input!")
            sys.exit(1)
    
    # Test the selected port
    success = test_port(port.device)
    
    print("\n" + "="*60)
    print("Diagnostic Summary:")
    print("="*60)
    
    if success:
        print("✓ Motor controller is responding correctly")
        print("\nNext steps:")
        print("1. Run: python main.py")
        print("2. Select port:", port.device)
        print("3. Click 'Connect'")
        print("4. Set frequency and amplitude")
        print("5. Click 'Start Motor'")
    else:
        print("✗ Motor controller not responding")
        print("\nPossible issues:")
        print("1. Motor controller not powered on")
        print("2. Wrong serial port selected")
        print("3. Wrong baudrate (expected: 115200)")
        print("4. USB/serial cable disconnected")
        print("5. Motor controller firmware issue")
        print("\nNext steps:")
        print("1. Check power supply to motor controller")
        print("2. Check USB cable connection")
        print("3. Try a different serial port")
        print("4. Verify motor controller supports 115200 baud")
    
    print("="*60 + "\n")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        sys.exit(0)
