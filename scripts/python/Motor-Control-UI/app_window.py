import sys
import time

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QSpinBox, QDoubleSpinBox, QComboBox,
    QTextEdit, QTabWidget, QGroupBox, QFormLayout, QStatusBar
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor

from serial_communicator import SerialCommunicator
from data_buffer import TelemetryBuffer

# Lazy imports for matplotlib - will be done in __init__ when QApplication exists
FigureCanvas = None
Figure = None
plt = None


def _init_matplotlib():
    """Initialize matplotlib backend - must be called after QApplication is created."""
    global FigureCanvas, Figure, plt
    if FigureCanvas is not None:
        return  # Already initialized
    
    import matplotlib
    matplotlib.use('Qt5Agg')
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FC
    from matplotlib.figure import Figure as Fig
    import matplotlib.pyplot as mp
    
    FigureCanvas = FC
    Figure = Fig
    plt = mp


from config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, REFRESH_RATE,
    FREQUENCY_MIN, FREQUENCY_MAX, FREQUENCY_DEFAULT,
    AMPLITUDE_MIN, AMPLITUDE_MAX, AMPLITUDE_DEFAULT,
    RPM_MIN, RPM_MAX, RPM_DEFAULT
)


class SignalEmitter(QObject):
    """Helper class to emit signals from threads."""
    data_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool, str)
    error_occurred = pyqtSignal(str)


class MotorControlUI(QMainWindow):
    """Main application window for motor control."""
    
    def __init__(self):
        super().__init__()
        
        # Initialize matplotlib backend (must be done after QApplication is created, before using Figure)
        _init_matplotlib()
        
        self.setWindowTitle("Motor Control UI - Induction Machine FOC")
        self.setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT)
        
        # Create signal emitter for thread-safe signals
        self.signal_emitter = SignalEmitter()
        self.signal_emitter.data_received.connect(self._on_data_received)
        self.signal_emitter.connection_changed.connect(self._on_connection_changed)
        self.signal_emitter.error_occurred.connect(self._on_error)
        
        # Initialize components
        self.serial_comm = SerialCommunicator()
        self.serial_comm.on_data_received = lambda data: self.signal_emitter.data_received.emit(data)
        self.serial_comm.on_connection_change = lambda status, msg: self.signal_emitter.connection_changed.emit(status, msg)
        self.serial_comm.on_error = lambda msg: self.signal_emitter.error_occurred.emit(msg)
        
        self.data_buffer = TelemetryBuffer()
        
        # UI state
        self.connected = False
        self.motor_running = False
        
        # Create UI
        self._create_ui()
        
        # Setup update timer for plots
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(REFRESH_RATE)
        
        # Status bar
        self.statusBar().showMessage("Ready. Select a serial port to connect.")
    
    def _create_ui(self):
        """Create the main UI layout."""
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        main_layout = QHBoxLayout(main_widget)
        
        # Left panel: Controls
        left_panel = self._create_control_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Right panel: Plots
        right_panel = self._create_plot_panel()
        main_layout.addWidget(right_panel, 2)
    
    def _create_control_panel(self) -> QWidget:
        """Create the control panel on the left."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Connection Group
        connection_group = self._create_connection_group()
        layout.addWidget(connection_group)
        
        # Control Group
        control_group = self._create_control_group()
        layout.addWidget(control_group)

        # FOC Control Group
        foc_group = self._create_foc_control_group()
        layout.addWidget(foc_group)
        
        # Status Group
        self.status_widget = self._create_status_group()
        layout.addWidget(self.status_widget)
        
        # Log/Messages Group
        log_group = self._create_log_group()
        layout.addWidget(log_group)
        
        layout.addStretch()
        return panel
    
    def _create_connection_group(self) -> QGroupBox:
        """Create connection control group."""
        group = QGroupBox("Connection")
        layout = QFormLayout()
        
        # Port selection
        port_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.addItems(self.serial_comm.list_ports())
        
        refresh_button = QPushButton("Refresh")
        refresh_button.clicked.connect(self._refresh_ports)
        port_layout.addWidget(self.port_combo)
        port_layout.addWidget(refresh_button)
        
        layout.addRow("Serial Port:", port_layout)
        
        # Connect/Disconnect buttons
        button_layout = QHBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self._on_connect)
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self._on_disconnect)
        self.disconnect_button.setEnabled(False)
        
        button_layout.addWidget(self.connect_button)
        button_layout.addWidget(self.disconnect_button)
        layout.addRow(button_layout)
        
        # Status indicator
        self.connection_status_label = QLabel("Disconnected")
        self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addRow("Status:", self.connection_status_label)
        
        group.setLayout(layout)
        return group
    
    def _create_control_group(self) -> QGroupBox:
        """Create motor control group."""
        group = QGroupBox("Motor Control")
        layout = QFormLayout()
        
        # Frequency control
        self.frequency_spin = QDoubleSpinBox()
        self.frequency_spin.setMinimum(FREQUENCY_MIN)
        self.frequency_spin.setMaximum(FREQUENCY_MAX)
        self.frequency_spin.setValue(FREQUENCY_DEFAULT)
        self.frequency_spin.setSuffix(" Hz")
        self.frequency_spin.setSingleStep(0.5)
        
        freq_apply_button = QPushButton("Set")
        freq_apply_button.clicked.connect(self._set_frequency)
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(self.frequency_spin)
        freq_layout.addWidget(freq_apply_button)
        layout.addRow("Frequency:", freq_layout)
        
        # Amplitude control
        self.amplitude_spin = QDoubleSpinBox()
        self.amplitude_spin.setMinimum(AMPLITUDE_MIN)
        self.amplitude_spin.setMaximum(AMPLITUDE_MAX)
        self.amplitude_spin.setValue(AMPLITUDE_DEFAULT)
        self.amplitude_spin.setSingleStep(0.1)
        self.amplitude_spin.setDecimals(2)
        
        amp_apply_button = QPushButton("Set")
        amp_apply_button.clicked.connect(self._set_amplitude)
        amp_layout = QHBoxLayout()
        amp_layout.addWidget(self.amplitude_spin)
        amp_layout.addWidget(amp_apply_button)
        layout.addRow("Amplitude:", amp_layout)
        
        # RPM control
        self.rpm_spin = QSpinBox()
        self.rpm_spin.setMinimum(RPM_MIN)
        self.rpm_spin.setMaximum(RPM_MAX)
        self.rpm_spin.setValue(RPM_DEFAULT)
        self.rpm_spin.setSuffix(" RPM")
        self.rpm_spin.setSingleStep(100)
        
        rpm_apply_button = QPushButton("Set")
        rpm_apply_button.clicked.connect(self._set_rpm)
        rpm_layout = QHBoxLayout()
        rpm_layout.addWidget(self.rpm_spin)
        rpm_layout.addWidget(rpm_apply_button)
        layout.addRow("Target RPM:", rpm_layout)
        
        # Motor control buttons
        button_layout = QHBoxLayout()
        self.start_button = QPushButton("Start Motor")
        self.start_button.setStyleSheet("background-color: #90EE90; font-weight: bold;")
        self.start_button.clicked.connect(self._start_motor)
        self.start_button.setEnabled(False)
        
        self.stop_button = QPushButton("Stop Motor")
        self.stop_button.setStyleSheet("background-color: #FFB6C1; font-weight: bold;")
        self.stop_button.clicked.connect(self._stop_motor)
        self.stop_button.setEnabled(False)
        
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        layout.addRow(button_layout)
        
        group.setLayout(layout)
        return group

    def _create_foc_control_group(self) -> QGroupBox:
        """Create FOC control group."""
        group = QGroupBox("FOC Control")
        layout = QFormLayout()

        # Mode selection
        mode_layout = QHBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["V/F", "FOC"])
        mode_apply_button = QPushButton("Set")
        mode_apply_button.clicked.connect(self._set_mode)
        mode_layout.addWidget(self.mode_combo)
        mode_layout.addWidget(mode_apply_button)
        layout.addRow("Mode:", mode_layout)

        # Id reference
        self.id_spin = QDoubleSpinBox()
        self.id_spin.setRange(-50.0, 50.0)
        self.id_spin.setDecimals(3)
        self.id_spin.setSingleStep(0.1)
        id_apply_button = QPushButton("Set")
        id_apply_button.clicked.connect(self._set_id_current)
        id_layout = QHBoxLayout()
        id_layout.addWidget(self.id_spin)
        id_layout.addWidget(id_apply_button)
        layout.addRow("Id Ref (A):", id_layout)

        # Iq reference
        self.iq_spin = QDoubleSpinBox()
        self.iq_spin.setRange(-50.0, 50.0)
        self.iq_spin.setDecimals(3)
        self.iq_spin.setSingleStep(0.1)
        iq_apply_button = QPushButton("Set")
        iq_apply_button.clicked.connect(self._set_iq_current)
        iq_layout = QHBoxLayout()
        iq_layout.addWidget(self.iq_spin)
        iq_layout.addWidget(iq_apply_button)
        layout.addRow("Iq Ref (A):", iq_layout)

        # Id PID tuning
        self.id_kp_spin = QDoubleSpinBox()
        self.id_ki_spin = QDoubleSpinBox()
        self.id_kd_spin = QDoubleSpinBox()
        for spin in (self.id_kp_spin, self.id_ki_spin, self.id_kd_spin):
            spin.setRange(0.0, 1000.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.1)
        id_pid_layout = QHBoxLayout()
        id_pid_layout.addWidget(QLabel("Kp"))
        id_pid_layout.addWidget(self.id_kp_spin)
        id_pid_layout.addWidget(QLabel("Ki"))
        id_pid_layout.addWidget(self.id_ki_spin)
        id_pid_layout.addWidget(QLabel("Kd"))
        id_pid_layout.addWidget(self.id_kd_spin)
        id_pid_button = QPushButton("Set")
        id_pid_button.clicked.connect(self._set_id_pid)
        id_pid_layout.addWidget(id_pid_button)
        layout.addRow("Id PID:", id_pid_layout)

        # Iq PID tuning
        self.iq_kp_spin = QDoubleSpinBox()
        self.iq_ki_spin = QDoubleSpinBox()
        self.iq_kd_spin = QDoubleSpinBox()
        for spin in (self.iq_kp_spin, self.iq_ki_spin, self.iq_kd_spin):
            spin.setRange(0.0, 1000.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.1)
        iq_pid_layout = QHBoxLayout()
        iq_pid_layout.addWidget(QLabel("Kp"))
        iq_pid_layout.addWidget(self.iq_kp_spin)
        iq_pid_layout.addWidget(QLabel("Ki"))
        iq_pid_layout.addWidget(self.iq_ki_spin)
        iq_pid_layout.addWidget(QLabel("Kd"))
        iq_pid_layout.addWidget(self.iq_kd_spin)
        iq_pid_button = QPushButton("Set")
        iq_pid_button.clicked.connect(self._set_iq_pid)
        iq_pid_layout.addWidget(iq_pid_button)
        layout.addRow("Iq PID:", iq_pid_layout)

        group.setLayout(layout)
        return group
    
    def _create_status_group(self) -> QGroupBox:
        """Create status display group."""
        group = QGroupBox("Current Status")
        layout = QFormLayout()
        
        # Speed
        self.speed_label = QLabel("0.00 RPM")
        self.speed_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addRow("Actual Speed:", self.speed_label)
        
        # Current
        self.current_label = QLabel("Ia: 0.00 A, Ib: 0.00 A")
        self.current_label.setFont(QFont("Arial", 10))
        layout.addRow("Phase Current:", self.current_label)
        
        # Applied Torque
        self.torque_applied_label = QLabel("0.00 N⋅m")
        self.torque_applied_label.setFont(QFont("Arial", 10))
        layout.addRow("Applied Torque:", self.torque_applied_label)
        
        # Electromagnetic Torque
        self.torque_em_label = QLabel("0.00 N⋅m")
        self.torque_em_label.setFont(QFont("Arial", 10))
        layout.addRow("EM Torque:", self.torque_em_label)
        
        group.setLayout(layout)
        return group
    
    def _create_log_group(self) -> QGroupBox:
        """Create message log group."""
        group = QGroupBox("Messages")
        layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setFont(QFont("Courier", 8))
        layout.addWidget(self.log_text)
        
        group.setLayout(layout)
        return group
    
    def _create_plot_panel(self) -> QWidget:
        """Create the plot panel on the right."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create tab widget for different plots
        tabs = QTabWidget()
        
        # Speed plot
        self.speed_canvas = FigureCanvas(Figure(figsize=(8, 3)))
        self.speed_ax = self.speed_canvas.figure.add_subplot(111)
        tabs.addTab(self.speed_canvas, "Speed (RPM)")
        
        # Torque plot
        self.torque_canvas = FigureCanvas(Figure(figsize=(8, 3)))
        self.torque_ax = self.torque_canvas.figure.add_subplot(111)
        tabs.addTab(self.torque_canvas, "Torque (N⋅m)")
        
        # Current plot
        self.current_canvas = FigureCanvas(Figure(figsize=(8, 3)))
        self.current_ax = self.current_canvas.figure.add_subplot(111)
        tabs.addTab(self.current_canvas, "Phase Currents (A)")

        # Clarke currents plot
        self.clarke_canvas = FigureCanvas(Figure(figsize=(8, 3)))
        self.clarke_ax = self.clarke_canvas.figure.add_subplot(111)
        tabs.addTab(self.clarke_canvas, "Clarke (A)")

        # Park currents plot
        self.park_canvas = FigureCanvas(Figure(figsize=(8, 3)))
        self.park_ax = self.park_canvas.figure.add_subplot(111)
        tabs.addTab(self.park_canvas, "Park (A)")

        # Theta mechanical plot
        self.theta_mech_canvas = FigureCanvas(Figure(figsize=(8, 3)))
        self.theta_mech_ax = self.theta_mech_canvas.figure.add_subplot(111)
        tabs.addTab(self.theta_mech_canvas, "Theta Mech (rad)")

        # Theta electrical plot
        self.theta_elec_canvas = FigureCanvas(Figure(figsize=(8, 3)))
        self.theta_elec_ax = self.theta_elec_canvas.figure.add_subplot(111)
        tabs.addTab(self.theta_elec_canvas, "Theta Elec (rad)")
        
        layout.addWidget(tabs)
        return panel
    
    # Connection Methods
    def _refresh_ports(self):
        """Refresh the list of available serial ports."""
        self.port_combo.clear()
        ports = self.serial_comm.list_ports()
        self.port_combo.addItems(ports)
        if ports:
            self._add_log_message(f"Found {len(ports)} port(s): {', '.join(ports)}")
        else:
            self._add_log_message("No serial ports found")
    
    def _on_connect(self):
        """Handle connect button click."""
        port = self.port_combo.currentText()
        if not port:
            self._add_log_message("ERROR: No port selected")
            return
        
        if self.serial_comm.connect(port):
            self.connected = True
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(True)
            self._add_log_message(f"✓ Connected to {port}")
        else:
            self._add_log_message(f"✗ Failed to connect to {port}")
    
    def _on_disconnect(self):
        """Handle disconnect button click."""
        self.serial_comm.disconnect()
        self.connected = False
        self.motor_running = False
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self._add_log_message("Disconnected")
    
    def _on_connection_changed(self, status: bool, message: str):
        """Handle connection status change."""
        if status:
            self.connection_status_label.setText("Connected")
            self.connection_status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.connection_status_label.setText("Disconnected")
            self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
        
        self._add_log_message(message)
    
    # Control Methods
    def _set_frequency(self):
        """Set motor frequency."""
        freq = self.frequency_spin.value()
        if self.serial_comm.set_frequency(freq):
            self.data_buffer.set_reference_values(frequency=freq)
            self._add_log_message(f"Frequency set to {freq} Hz")
        else:
            self._add_log_message("ERROR: Failed to set frequency")
    
    def _set_amplitude(self):
        """Set inverter amplitude."""
        amp = self.amplitude_spin.value()
        if self.serial_comm.set_amplitude(amp):
            self.data_buffer.set_reference_values(amplitude=amp)
            self._add_log_message(f"Amplitude set to {amp}")
        else:
            self._add_log_message("ERROR: Failed to set amplitude")
    
    def _set_rpm(self):
        """Set target RPM."""
        rpm = self.rpm_spin.value()
        if self.serial_comm.set_rpm(rpm):
            self.data_buffer.set_reference_values(rpm=rpm)
            self._add_log_message(f"Target RPM set to {rpm}")
        else:
            self._add_log_message("ERROR: Failed to set RPM")

    def _set_mode(self):
        """Set control mode (V/F or FOC)."""
        mode = self.mode_combo.currentText()
        if mode == "FOC":
            ok = self.serial_comm.set_mode_foc()
        else:
            ok = self.serial_comm.set_mode_vf()

        if ok:
            self._add_log_message(f"Control mode set to {mode}")
        else:
            self._add_log_message("ERROR: Failed to set control mode")

    def _set_id_current(self):
        """Set Id reference current."""
        id_ref = self.id_spin.value()
        if self.serial_comm.set_id_current(id_ref):
            self._add_log_message(f"Id reference set to {id_ref} A")
        else:
            self._add_log_message("ERROR: Failed to set Id reference")

    def _set_iq_current(self):
        """Set Iq reference current."""
        iq_ref = self.iq_spin.value()
        if self.serial_comm.set_iq_current(iq_ref):
            self._add_log_message(f"Iq reference set to {iq_ref} A")
        else:
            self._add_log_message("ERROR: Failed to set Iq reference")

    def _set_id_pid(self):
        """Tune Id PID controller."""
        kp = self.id_kp_spin.value()
        ki = self.id_ki_spin.value()
        kd = self.id_kd_spin.value()
        if self.serial_comm.set_id_pid(kp, ki, kd):
            self._add_log_message(f"Id PID set: Kp={kp}, Ki={ki}, Kd={kd}")
        else:
            self._add_log_message("ERROR: Failed to set Id PID")

    def _set_iq_pid(self):
        """Tune Iq PID controller."""
        kp = self.iq_kp_spin.value()
        ki = self.iq_ki_spin.value()
        kd = self.iq_kd_spin.value()
        if self.serial_comm.set_iq_pid(kp, ki, kd):
            self._add_log_message(f"Iq PID set: Kp={kp}, Ki={ki}, Kd={kd}")
        else:
            self._add_log_message("ERROR: Failed to set Iq PID")
    
    def _start_motor(self):
        """Start the motor."""
        if self.serial_comm.start_motor():
            self.motor_running = True
            
            # Sync current UI values to data buffer
            freq = self.frequency_spin.value()
            amp = self.amplitude_spin.value()
            rpm = self.rpm_spin.value()
            
            self.data_buffer.set_reference_values(frequency=freq, amplitude=amp, rpm=rpm)
            
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self._add_log_message(f"Motor started (Target: {rpm} RPM @ {freq} Hz)")
        else:
            self._add_log_message("ERROR: Failed to start motor")
    
    def _stop_motor(self):
        """Stop the motor."""
        if self.serial_comm.stop_motor():
            self.motor_running = False
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self._add_log_message("Motor stopped")
        else:
            self._add_log_message("ERROR: Failed to stop motor")
    
    # Data Methods
    def _on_data_received(self, data: dict):
        """Handle received telemetry data."""
        if 'speed' in data and 'ia' in data and 'ib' in data:
            self.data_buffer.add_telemetry(
                data['speed'],
                data['ia'],
                data['ib'],
                i_alpha=data.get('i_alpha'),
                i_beta=data.get('i_beta'),
                id_current=data.get('id'),
                iq_current=data.get('iq'),
                theta_mechanical=data.get('theta_mechanical'),
                theta_electrical=data.get('theta_electrical'),
            )
            self._update_status_display(data)
    
    def _update_status_display(self, data: dict):
        """Update the status display with latest values."""
        latest = self.data_buffer.get_latest_values()
        
        self.speed_label.setText(f"{latest['speed']:.2f} RPM")
        self.current_label.setText(f"Ia: {latest['ia']:.3f} A, Ib: {latest['ib']:.3f} A")
        self.torque_applied_label.setText(f"{latest['torque_applied']:.3f} N⋅m")
        self.torque_em_label.setText(f"{latest['torque_electromagnetic']:.3f} N⋅m")
    
    def _update_plots(self):
        """Update all plots with latest data."""
        # Only plot real data from motor controller
        plot_data = self.data_buffer.get_plot_data()
        
        if len(plot_data['time']) > 0:
            self._plot_speed(plot_data)
            self._plot_torque(plot_data)
            self._plot_current(plot_data)
            self._plot_clarke(plot_data)
            self._plot_park(plot_data)
            self._plot_theta_mechanical(plot_data)
            self._plot_theta_electrical(plot_data)
    
    def _plot_speed(self, data: dict):
        """Plot speed data."""
        self.speed_ax.clear()
        
        time = data['time']
        speed_actual = data['speed_actual']
        speed_ref = data['speed_reference']
        
        self.speed_ax.plot(time, speed_actual, 'b-', label='Actual Speed', linewidth=1.5)
        self.speed_ax.plot(time, speed_ref, 'r--', label='Reference Speed', linewidth=1.5)
        
        self.speed_ax.set_xlabel('Time (s)')
        self.speed_ax.set_ylabel('Speed (RPM)')
        self.speed_ax.set_title('Motor Speed')
        self.speed_ax.legend(loc='best')
        self.speed_ax.grid(True, alpha=0.3)
        
        self.speed_canvas.draw()
    
    def _plot_torque(self, data: dict):
        """Plot torque data."""
        self.torque_ax.clear()
        
        time = data['time']
        t_applied = data['torque_applied']
        t_em = data['torque_electromagnetic']
        
        self.torque_ax.plot(time, t_applied, 'g-', label='Applied Torque', linewidth=1.5)
        self.torque_ax.plot(time, t_em, 'm-', label='Electromagnetic Torque', linewidth=1.5)
        
        self.torque_ax.set_xlabel('Time (s)')
        self.torque_ax.set_ylabel('Torque (N⋅m)')
        self.torque_ax.set_title('Motor Torque')
        self.torque_ax.legend(loc='best')
        self.torque_ax.grid(True, alpha=0.3)
        
        self.torque_canvas.draw()
    
    def _plot_current(self, data: dict):
        """Plot current data."""
        self.current_ax.clear()
        
        time = data['time']
        ia = data['current_ia']
        ib = data['current_ib']
        
        self.current_ax.plot(time, ia, 'c-', label='Phase A (Ia)', linewidth=1.5)
        self.current_ax.plot(time, ib, 'y-', label='Phase B (Ib)', linewidth=1.5)
        
        self.current_ax.set_xlabel('Time (s)')
        self.current_ax.set_ylabel('Current (A)')
        self.current_ax.set_title('Phase Currents')
        self.current_ax.legend(loc='best')
        self.current_ax.grid(True, alpha=0.3)
        
        self.current_canvas.draw()

    def _plot_clarke(self, data: dict):
        """Plot Clarke (alpha/beta) currents."""
        self.clarke_ax.clear()

        time = data['time']
        i_alpha = data['current_alpha']
        i_beta = data['current_beta']

        self.clarke_ax.plot(time, i_alpha, 'b-', label='Ialpha', linewidth=1.5)
        self.clarke_ax.plot(time, i_beta, 'r-', label='Ibeta', linewidth=1.5)

        self.clarke_ax.set_xlabel('Time (s)')
        self.clarke_ax.set_ylabel('Current (A)')
        self.clarke_ax.set_title('Clarke Currents')
        self.clarke_ax.legend(loc='best')
        self.clarke_ax.grid(True, alpha=0.3)

        self.clarke_canvas.draw()

    def _plot_park(self, data: dict):
        """Plot Park (d/q) currents."""
        self.park_ax.clear()

        time = data['time']
        i_d = data['current_id']
        i_q = data['current_iq']

        self.park_ax.plot(time, i_d, 'g-', label='Id', linewidth=1.5)
        self.park_ax.plot(time, i_q, 'm-', label='Iq', linewidth=1.5)

        self.park_ax.set_xlabel('Time (s)')
        self.park_ax.set_ylabel('Current (A)')
        self.park_ax.set_title('Park Currents')
        self.park_ax.legend(loc='best')
        self.park_ax.grid(True, alpha=0.3)

        self.park_canvas.draw()

    def _plot_theta_mechanical(self, data: dict):
        """Plot mechanical angle (theta_m)."""
        self.theta_mech_ax.clear()

        time = data['time']
        theta_m = data['theta_mechanical']

        self.theta_mech_ax.plot(time, theta_m, 'b-', label='Theta Mechanical', linewidth=1.5)

        self.theta_mech_ax.set_xlabel('Time (s)')
        self.theta_mech_ax.set_ylabel('Theta (rad)')
        self.theta_mech_ax.set_title('Mechanical Angle')
        self.theta_mech_ax.legend(loc='best')
        self.theta_mech_ax.grid(True, alpha=0.3)

        self.theta_mech_canvas.draw()

    def _plot_theta_electrical(self, data: dict):
        """Plot electrical angle (theta_e)."""
        self.theta_elec_ax.clear()

        time = data['time']
        theta_e = data['theta_electrical']

        self.theta_elec_ax.plot(time, theta_e, 'r-', label='Theta Electrical', linewidth=1.5)

        self.theta_elec_ax.set_xlabel('Time (s)')
        self.theta_elec_ax.set_ylabel('Theta (rad)')
        self.theta_elec_ax.set_title('Electrical Angle')
        self.theta_elec_ax.legend(loc='best')
        self.theta_elec_ax.grid(True, alpha=0.3)

        self.theta_elec_canvas.draw()
    
    def _on_error(self, message: str):
        """Handle error messages."""
        self._add_log_message(f"⚠ {message}")
    
    def _add_log_message(self, message: str):
        """Add a message to the log."""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        # Auto-scroll to bottom
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def closeEvent(self, event):
        """Handle window close event."""
        if self.connected:
            self.serial_comm.disconnect()
        event.accept()
