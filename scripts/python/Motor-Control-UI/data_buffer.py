import numpy as np
from collections import deque
from typing import Optional
from config import BUFFER_SIZE
import time


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
        self.theta_e_integ_values = deque(maxlen=buffer_size)  # integrated electrical angle (rad)

        # Speed reference for overlay on plot
        self.speed_references = deque(maxlen=buffer_size)
        self.reference_speed = 0.0  # rad/s

    def add_telemetry(self, id_val: float, iq_val: float, vbus: float,
                      omega_m: float, ia: float, ib: float, ic: float,
                      theta_e: float = 0.0, theta_e_integ: float = 0.0):
        current_time = time.time()
        self.timestamps.append(current_time)
        self.id_currents.append(id_val)
        self.iq_currents.append(iq_val)
        self.vbus_values.append(vbus)
        self.omega_m_values.append(omega_m)
        self.currents_ia.append(ia)
        self.currents_ib.append(ib)
        self.currents_ic.append(ic)
        self.theta_e_values.append(theta_e)
        self.theta_e_integ_values.append(theta_e_integ)
        self.speed_references.append(self.reference_speed)

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
                'theta_e_integ': empty,
            }

        time_array = np.array(list(self.timestamps))
        time_relative = time_array - time_array[0]

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
            'theta_e_integ': np.array(list(self.theta_e_integ_values)),
        }

    def get_latest_values(self) -> dict:
        if len(self.timestamps) == 0:
            return {
                'omega_m': 0.0, 'id': 0.0, 'iq': 0.0,
                'vbus': 0.0, 'ia': 0.0, 'ib': 0.0, 'ic': 0.0,
                'theta_e': 0.0, 'theta_e_integ': 0.0,
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
            'theta_e_integ': self.theta_e_integ_values[-1],
        }

    def clear(self):
        self.timestamps.clear()
        self.id_currents.clear()
        self.iq_currents.clear()
        self.vbus_values.clear()
        self.omega_m_values.clear()
        self.currents_ia.clear()
        self.currents_ib.clear()
        self.currents_ic.clear()
        self.theta_e_values.clear()
        self.theta_e_integ_values.clear()
        self.speed_references.clear()
