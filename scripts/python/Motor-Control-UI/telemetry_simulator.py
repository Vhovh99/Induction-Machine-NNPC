"""
Motor telemetry simulator for development and testing

Provides simulated Speed and Current data based on control inputs
when real telemetry isn't available or is too slow.
"""

import time
import math


class TelemetrySimulator:
    """Simulates motor telemetry data based on control commands."""
    
    def __init__(self):
        """Initialize simulator state."""
        self.current_speed = 0.0  # RPM
        self.current_ia = 0.0     # Amps
        self.current_ib = 0.0     # Amps
        
        # Control inputs
        self.target_frequency = 0.0  # Hz
        self.target_amplitude = 0.0  # 0.0-1.0
        self.target_rpm = 0.0        # Target RPM (takes precedence if set)
        self.motor_running = False
        
        # Simulation parameters
        self.speed_ramp_rate = 200.0  # RPM/sec
        self.motor_pole_pairs = 2
        self.last_update = time.time()
        self.phase = 0.0
        
    def set_control_input(self, frequency: float, amplitude: float, motor_running: bool):
        """
        Set the control inputs (from UI commands).
        
        Args:
            frequency: Target frequency in Hz
            amplitude: Target amplitude (0.0-1.0)
            motor_running: Motor running state
        """
        self.target_frequency = frequency
        self.target_amplitude = amplitude
        self.motor_running = motor_running
        
    def update(self) -> dict:
        """
        Update simulated telemetry.
        
        Returns:
            Dictionary with  'speed', 'ia', 'ib' keys
        """
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        
        if not self.motor_running:
            # Decelerate to stop
            if abs(self.current_speed) > 1.0:
                self.current_speed *= (1.0 - 0.1 * dt)
            else:
                self.current_speed = 0.0
                self.current_ia = 0.0
                self.current_ib = 0.0
        else:
            # Calculate target speed: prefer explicit target_rpm if set, else use frequency
            if self.target_rpm > 0:
                # Use explicit target RPM setting
                target_speed = self.target_rpm
            else:
                # Calculate from frequency
                # Synchronous speed = 120 * f / pole_pairs
                target_speed = abs((120.0 * self.target_frequency) / self.motor_pole_pairs)
            
            # Ramp speed towards target
            speed_error = target_speed - self.current_speed
            speed_change = self.speed_ramp_rate * dt
            
            if abs(speed_error) < speed_change:
                self.current_speed = target_speed
            else:
                self.current_speed += math.copysign(speed_change, speed_error)
            
            # Ensure speed never goes negative
            self.current_speed = max(0, self.current_speed)
            
            # Simulate phase currents (3-phase AC)
            # Current magnitude proportional to amplitude and slip
            current_mag = self.target_amplitude * 2.0  # Max ~2A at amplitude=1.0
            
            # Add slip-related current (load-dependent)
            slip_current = (target_speed - max(self.current_speed, 0)) / (target_speed + 0.1)
            current_mag += slip_current * 0.5
            
            # Generate 3-phase currents (simulating 2 phases)
            self.phase += 2.0 * math.pi * self.target_frequency * dt
            self.current_ia = current_mag * math.sin(self.phase)
            self.current_ib = current_mag * math.sin(self.phase - 2.0 * math.pi / 3.0)
        
        return {
            'speed': self.current_speed,
            'ia': self.current_ia,
            'ib': self.current_ib,
            'timestamp': now,
        }
    
    def get_current_values(self) -> dict:
        """Get current simulator state without updating."""
        return {
            'speed': self.current_speed,
            'ia': self.current_ia,
            'ib': self.current_ib,
        }
