"""
Motor telemetry simulator for development and testing.

Generates simulated telemetry matching the firmware binary protocol
telemetry packet: id, iq, vbus, omega_m, ia, ib, ic, theta_e, theta_e_integ.
"""

import time
import math


class TelemetrySimulator:
    """Simulates motor telemetry data matching firmware Telemetry_Packet_t."""

    def __init__(self):
        self.omega_m = 0.0      # mechanical speed  rad/s
        self.speed_ref = 0.0    # speed reference    rad/s
        self.id_ref = 0.4       # d-axis ref current A
        self.motor_state = 0    # 0=IDLE, 1=FLUX_BUILD, 2=RUNNING

        self.tau = 0.5          # speed time-constant
        self.last_update = time.time()
        self.phase = 0.0

    def set_control_input(self, speed_ref: float, id_ref: float, motor_running: bool):
        self.speed_ref = speed_ref
        self.id_ref = id_ref
        if motor_running and self.motor_state == 0:
            self.motor_state = 1
            self._flux_start = time.time()
        elif not motor_running:
            self.motor_state = 0

    def update(self) -> dict:
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        # State transitions
        if self.motor_state == 1 and now - self._flux_start >= 0.2:
            self.motor_state = 2

        if self.motor_state == 2:
            error = self.speed_ref - self.omega_m
            self.omega_m += (error / self.tau) * dt
        elif self.motor_state == 0:
            self.omega_m *= (1.0 - 2.0 * dt)
            if abs(self.omega_m) < 0.01:
                self.omega_m = 0.0

        self.phase += abs(self.omega_m) * dt

        cur_mag = abs(self.omega_m) * 0.01 + self.id_ref
        id_val = self.id_ref * 0.95
        iq_val = self.omega_m * 0.005 if self.motor_state == 2 else 0.0
        vbus = 24.0 + math.sin(now * 0.1) * 0.3
        ia = cur_mag * math.sin(self.phase)
        ib = cur_mag * math.sin(self.phase + 2.094)
        ic = cur_mag * math.sin(self.phase + 4.189)
        theta_e = math.fmod(self.phase * 2.0, 2.0 * math.pi)
        theta_e_integ = math.fmod(self.phase * 2.0 + 0.05, 2.0 * math.pi)

        return {
            'id': id_val,
            'iq': iq_val,
            'vbus': vbus,
            'omega_m': self.omega_m,
            'ia': ia,
            'ib': ib,
            'ic': ic,
            'theta_e': theta_e,
            'theta_e_integ': theta_e_integ,
            'timestamp': now,
        }
