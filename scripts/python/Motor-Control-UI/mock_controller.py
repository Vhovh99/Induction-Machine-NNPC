#!/usr/bin/env python3
"""
Mock Motor Controller Simulator

This script simulates a motor controller for testing the UI without a physical motor.
Can be used with a virtual serial port pair or as a reference implementation.

Usage:
    python mock_controller.py /dev/ttyVCOM0
    # Then connect the UI to /dev/ttyVCOM1
"""

import serial
import time
import math
import argparse
import numpy as np
from threading import Thread, Event


class MockMotorController:
    """Simulates a motor controller with realistic dynamics."""
    
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        
        # Motor state
        self.speed = 0.0  # RPM
        self.target_rpm = 0.0
        self.frequency = 50.0  # Hz
        self.amplitude = 0.0
        
        # Motor dynamics
        self.motor_running = False
        self.tau = 0.5  # Time constant for speed response (seconds)
        self.last_update = time.time()
        
        # Phase currents (simulated)
        self.phase = 0.0
        self.ia = 0.0
        self.ib = 0.0
    
    def connect(self):
        """Open the serial port."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            print(f"✓ Mock controller listening on {self.port}")
            return True
        except Exception as e:
            print(f"✗ Failed to open {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close the serial port."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"✓ Closed {self.port}")
    
    def run(self):
        """Main loop for the mock controller."""
        self.running = True
        telemetry_timer = 0
        telemetry_interval = 0.1  # 100ms
        
        print("Mock controller simulation started")
        print("Waiting for commands...")
        
        while self.running:
            # Read commands
            if self.serial.in_waiting > 0:
                try:
                    line = self.serial.readline().decode('ascii').strip()
                    if line:
                        self._process_command(line)
                except Exception as e:
                    print(f"Error reading command: {e}")
            
            # Update motor dynamics
            self._update_motor_dynamics()
            
            # Send telemetry periodically
            telemetry_timer += 0.01
            if telemetry_timer >= telemetry_interval:
                self._send_telemetry()
                telemetry_timer = 0
            
            time.sleep(0.01)
    
    def _process_command(self, command):
        """Process incoming commands."""
        parts = command.split()
        if not parts:
            return
        
        cmd = parts[0]
        
        try:
            if cmd == 'F' and len(parts) > 1:
                # Set frequency
                self.frequency = float(parts[1])
                print(f"  SET FREQUENCY: {self.frequency} Hz")
            
            elif cmd == 'A' and len(parts) > 1:
                # Set amplitude
                self.amplitude = float(parts[1])
                print(f"  SET AMPLITUDE: {self.amplitude}")
            
            elif cmd == 'R' and len(parts) > 1:
                # Set RPM
                self.target_rpm = float(parts[1])
                print(f"  SET TARGET RPM: {self.target_rpm}")
            
            elif cmd == 'S':
                # Start motor
                self.motor_running = True
                print(f"  MOTOR START")
            
            elif cmd == 'X':
                # Stop motor
                self.motor_running = False
                self.speed = 0.0
                print(f"  MOTOR STOP")
            
            else:
                print(f"  Unknown command: {command}")
        
        except Exception as e:
            print(f"  Error processing command: {e}")
    
    def _update_motor_dynamics(self):
        """Update motor speed and current based on control inputs."""
        dt = time.time() - self.last_update
        self.last_update = time.time()
        
        if not self.motor_running:
            # Decelerate to rest
            self.speed *= 0.98
            if abs(self.speed) < 1.0:
                self.speed = 0.0
        else:
            # Calculate target speed from frequency
            # Assuming: speed ≈ 120 * frequency / pole_pairs
            pole_pairs = 2
            base_speed = (120 * self.frequency) / pole_pairs
            target_speed = base_speed * self.amplitude
            
            # First-order lag to simulate motor response
            if dt > 0:
                error = target_speed - self.speed
                self.speed += (error / self.tau) * dt
        
        # Limit speed
        self.speed = max(-3000, min(3000, self.speed))
        
        # Simulate phase currents (sinusoidal with modulation)
        self.phase += 2 * math.pi * self.frequency * dt
        
        # Current magnitude proportional to amplitude
        current_magnitude = self.amplitude * 2.0
        
        # Three-phase currents (simulating two phases)
        self.ia = current_magnitude * math.sin(self.phase)
        self.ib = current_magnitude * math.sin(self.phase + 2 * math.pi / 3)
    
    def _send_telemetry(self):
        """Send telemetry data to the host."""
        try:
            # Format: SPD:xxxx.xx CUR:x.xx,x.xx
            telemetry = f"SPD:{self.speed:.2f} CUR:{self.ia:.2f},{self.ib:.2f}\n"
            self.serial.write(telemetry.encode('ascii'))
        except Exception as e:
            print(f"Error sending telemetry: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Mock Motor Controller Simulator'
    )
    parser.add_argument(
        'port',
        help='Serial port to listen on (e.g., COM3, /dev/ttyUSB0)'
    )
    parser.add_argument(
        '--baudrate',
        type=int,
        default=115200,
        help='Serial baudrate (default: 115200)'
    )
    
    args = parser.parse_args()
    
    # Create and run controller
    controller = MockMotorController(args.port, args.baudrate)
    
    if controller.connect():
        try:
            controller.run()
        except KeyboardInterrupt:
            print("\n\nShutting down...")
        finally:
            controller.disconnect()
    else:
        print("Failed to start mock controller")


if __name__ == '__main__':
    main()
