import numpy as np
from collections import deque
from typing import Optional
from config import BUFFER_SIZE, PWM_FREQUENCY, TELEMETRY_DIV_DEFAULT


class TelemetryBuffer:
    """Maintains circular buffers for telemetry data matching firmware telemetry packet."""

    def __init__(self, buffer_size: int = BUFFER_SIZE):
        self.buffer_size = buffer_size

        # Circular buffers matching firmware Telemetry_Packet_t fields
        self.timestamps = deque(maxlen=buffer_size)
        self.id_currents = deque(maxlen=buffer_size)       # d-axis current (A)
        self.iq_currents = deque(maxlen=buffer_size)       # q-axis current (A)
        self.vbus_values = deque(maxlen=buffer_size)       # DC bus voltage (V)
        self.omega_m_values = deque(maxlen=buffer_size)    # mechanical speed (rad/s)
        self.currents_ia = deque(maxlen=buffer_size)       # phase A current (A)
        self.currents_ib = deque(maxlen=buffer_size)       # phase B current (A)
        self.currents_ic = deque(maxlen=buffer_size)       # phase C current (A)
        self.theta_e_values = deque(maxlen=buffer_size)     # electrical angle (rad)
        self.torque_e_values = deque(maxlen=buffer_size)     # estimated electromagnetic torque (N·m)

        # Speed reference for overlay on plot
        self.speed_references = deque(maxlen=buffer_size)
        self.reference_speed = 0.0  # rad/s

        # Monotonic sample counter — avoids batched-read timestamp artifact:
        # multiple packets received in the same serial.read() call would all get
        # identical time.time() values, causing vertical spikes in the plot.
        self._sample_idx = 0
        self._sample_period = TELEMETRY_DIV_DEFAULT / PWM_FREQUENCY  # seconds per sample

    def add_telemetry(self, id_val: float, iq_val: float, vbus: float,
                      omega_m: float, ia: float, ib: float, ic: float,
                      theta_e: float = 0.0, torque_e: float = 0.0):
        self.timestamps.append(self._sample_idx)
        self._sample_idx += 1
        self.id_currents.append(id_val)
        self.iq_currents.append(iq_val)
        self.vbus_values.append(vbus)
        self.omega_m_values.append(omega_m)
        self.currents_ia.append(ia)
        self.currents_ib.append(ib)
        self.currents_ic.append(ic)
        self.theta_e_values.append(theta_e)
        self.torque_e_values.append(torque_e)
        self.speed_references.append(self.reference_speed)

    def set_telemetry_divider(self, divider: int):
        """Update the expected sample period when the telemetry divider changes."""
        if divider > 0:
            self._sample_period = divider / PWM_FREQUENCY

    def set_speed_reference(self, omega_ref: float):
        self.reference_speed = omega_ref

    def get_plot_data(self) -> dict:
        if len(self.timestamps) == 0:
            empty = np.array([])
            return {
                'time': empty,
                'omega_m': empty,
                'speed_reference': empty,
                'id': empty,
                'iq': empty,
                'vbus': empty,
                'ia': empty,
                'ib': empty,
                'ic': empty,
                'theta_e': empty,
                'torque_e': empty,
            }

        indices = np.array(list(self.timestamps))
        time_relative = (indices - indices[0]) * self._sample_period

        return {
            'time': time_relative,
            'omega_m': np.array(list(self.omega_m_values)),
            'speed_reference': np.array(list(self.speed_references)),
            'id': np.array(list(self.id_currents)),
            'iq': np.array(list(self.iq_currents)),
            'vbus': np.array(list(self.vbus_values)),
            'ia': np.array(list(self.currents_ia)),
            'ib': np.array(list(self.currents_ib)),
            'ic': np.array(list(self.currents_ic)),
            'theta_e': np.array(list(self.theta_e_values)),
            'torque_e': np.array(list(self.torque_e_values)),
        }

    def get_latest_values(self) -> dict:
        if len(self.timestamps) == 0:
            return {
                'omega_m': 0.0, 'id': 0.0, 'iq': 0.0,
                'vbus': 0.0, 'ia': 0.0, 'ib': 0.0, 'ic': 0.0,
                'theta_e': 0.0, 'torque_e': 0.0,
            }
        return {
            'omega_m': self.omega_m_values[-1],
            'id': self.id_currents[-1],
            'iq': self.iq_currents[-1],
            'vbus': self.vbus_values[-1],
            'ia': self.currents_ia[-1],
            'ib': self.currents_ib[-1],
            'ic': self.currents_ic[-1],
            'theta_e': self.theta_e_values[-1],
            'torque_e': self.torque_e_values[-1],
        }

    def clear(self):
        self.timestamps.clear()
        self._sample_idx = 0
        self.id_currents.clear()
        self.iq_currents.clear()
        self.vbus_values.clear()
        self.omega_m_values.clear()
        self.currents_ia.clear()
        self.currents_ib.clear()
        self.currents_ic.clear()
        self.theta_e_values.clear()
        self.torque_e_values.clear()
        self.speed_references.clear()
