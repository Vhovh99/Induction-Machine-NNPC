import numpy as np
from collections import deque
from typing import Optional, Tuple
from config import BUFFER_SIZE, MOTOR_POLE_PAIRS, MOTOR_MOMENT_OF_INERTIA, FRICTION_COEFFICIENT
import time


class TelemetryBuffer:
    """Maintains circular buffers for telemetry data and calculates derived values."""
    
    def __init__(self, buffer_size: int = BUFFER_SIZE):
        self.buffer_size = buffer_size
        
        # Circular buffers for telemetry
        self.timestamps = deque(maxlen=buffer_size)
        self.speeds = deque(maxlen=buffer_size)
        self.speeds_reference = deque(maxlen=buffer_size)
        self.currents_ia = deque(maxlen=buffer_size)
        self.currents_ib = deque(maxlen=buffer_size)
        self.torques_applied = deque(maxlen=buffer_size)
        self.torques_electromagnetic = deque(maxlen=buffer_size)
        
        # Current control values
        self.reference_frequency = 50.0  # Hz
        self.reference_amplitude = 0.5
        self.reference_rpm = 1500
        
        # Last values for derivative calculation
        self.last_speed = 0.0
        self.last_timestamp = time.time()
    
    def add_telemetry(self, speed: float, ia: float, ib: float):
        """Add a new telemetry sample."""
        current_time = time.time()
        dt = current_time - self.last_timestamp
        
        self.timestamps.append(current_time)
        # Ensure speed is never negative (absolute value for display)
        self.speeds.append(abs(speed))
        
        # Use the reference RPM directly if set, otherwise calculate from frequency
        if self.reference_rpm > 0:
            reference_speed = self.reference_rpm
        else:
            # Fallback: Calculate reference speed from frequency
            # Assuming synchronous speed = 120 * f / P, where P is pole pairs
            reference_speed = (120 * self.reference_frequency) / MOTOR_POLE_PAIRS
        self.speeds_reference.append(reference_speed)
        
        # Store currents
        self.currents_ia.append(ia)
        self.currents_ib.append(ib)
        
        # Calculate torques
        applied_torque = self._calculate_applied_torque(ia, ib)
        electromagnetic_torque = self._calculate_electromagnetic_torque(speed, applied_torque, dt)
        
        self.torques_applied.append(applied_torque)
        self.torques_electromagnetic.append(electromagnetic_torque)
        
        self.last_speed = speed
        self.last_timestamp = current_time
    
    def set_reference_values(self, frequency: float = None, amplitude: float = None, rpm: float = None):
        """Set reference values for control."""
        if frequency is not None:
            self.reference_frequency = frequency
        if amplitude is not None:
            self.reference_amplitude = amplitude
        if rpm is not None:
            self.reference_rpm = rpm
    
    def _calculate_applied_torque(self, ia: float, ib: float) -> float:
        """
        Calculate the applied torque from phase currents.
        Simplified model: T = K * sqrt(Ia^2 + Ib^2)
        where K is a motor constant to be calibrated.
        """
        # Motor constant (should be calibrated based on motor spec)
        K_motor = 2.5  # N*m/A (example value)
        
        current_magnitude = np.sqrt(ia**2 + ib**2)
        return K_motor * current_magnitude
    
    def _calculate_electromagnetic_torque(self, current_speed: float, applied_torque: float, dt: float) -> float:
        """
        Estimate electromagnetic torque using the dynamic equation:
        T_em - T_friction = J * dω/dt
        where J is moment of inertia, ω is angular velocity.
        """
        if dt <= 0:
            return applied_torque
        
        # Convert RPM to rad/s for calculations
        omega_current = current_speed * 2 * np.pi / 60
        omega_last = self.last_speed * 2 * np.pi / 60
        
        # Angular acceleration
        dw_dt = (omega_current - omega_last) / dt if dt > 0 else 0
        
        # Friction torque (proportional to speed)
        T_friction = FRICTION_COEFFICIENT * current_speed / 60
        
        # From: T_em = J * dω/dt + T_friction
        T_em = MOTOR_MOMENT_OF_INERTIA * dw_dt + T_friction
        
        return T_em
    
    def get_plot_data(self) -> dict:
        """Get data suitable for plotting."""
        # Convert deques to arrays for easier manipulation
        if len(self.timestamps) == 0:
            return {
                'time': np.array([]),
                'speed_actual': np.array([]),
                'speed_reference': np.array([]),
                'torque_applied': np.array([]),
                'torque_electromagnetic': np.array([]),
                'current_ia': np.array([]),
                'current_ib': np.array([]),
            }
        
        # Create time array relative to start (for better x-axis readability)
        time_array = np.array(list(self.timestamps))
        if len(time_array) > 0:
            time_relative = time_array - time_array[0]
        else:
            time_relative = np.array([])
        
        return {
            'time': time_relative,
            'speed_actual': np.array(list(self.speeds)),
            'speed_reference': np.array(list(self.speeds_reference)),
            'torque_applied': np.array(list(self.torques_applied)),
            'torque_electromagnetic': np.array(list(self.torques_electromagnetic)),
            'current_ia': np.array(list(self.currents_ia)),
            'current_ib': np.array(list(self.currents_ib)),
        }
    
    def get_latest_values(self) -> dict:
        """Get the latest measured values."""
        if len(self.speeds) == 0:
            return {
                'speed': 0.0,
                'ia': 0.0,
                'ib': 0.0,
                'torque_applied': 0.0,
                'torque_electromagnetic': 0.0,
            }
        
        return {
            'speed': self.speeds[-1],
            'ia': self.currents_ia[-1],
            'ib': self.currents_ib[-1],
            'torque_applied': self.torques_applied[-1],
            'torque_electromagnetic': self.torques_electromagnetic[-1],
        }
    
    def clear(self):
        """Clear all buffers."""
        self.timestamps.clear()
        self.speeds.clear()
        self.speeds_reference.clear()
        self.currents_ia.clear()
        self.currents_ib.clear()
        self.torques_applied.clear()
        self.torques_electromagnetic.clear()
