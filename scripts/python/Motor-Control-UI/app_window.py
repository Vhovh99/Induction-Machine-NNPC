import sys
import time
import csv
import os

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QScrollArea,
    QPushButton, QLabel, QSpinBox, QDoubleSpinBox, QComboBox,
    QTextEdit, QTabWidget, QGroupBox, QFormLayout, QStatusBar,
    QLineEdit, QFileDialog
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor

from serial_communicator import SerialCommunicator, NACK_ERRORS, MOTOR_STATES
from data_buffer import TelemetryBuffer

# Lazy imports for matplotlib - will be done in __init__ when QApplication exists
FigureCanvas = None
Figure = None
plt = None
NavigationToolbar = None


def _init_matplotlib():
    """Initialize matplotlib backend - must be called after QApplication is created."""
    global FigureCanvas, Figure, plt, NavigationToolbar
    if FigureCanvas is not None:
        return  # Already initialized
    
    import matplotlib
    matplotlib.use('Qt5Agg')
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FC
    from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NT
    from matplotlib.figure import Figure as Fig
    import matplotlib.pyplot as mp
    
    FigureCanvas = FC
    NavigationToolbar = NT
    Figure = Fig
    plt = mp


from config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, REFRESH_RATE,
    SPEED_REF_MIN, SPEED_REF_MAX, SPEED_REF_DEFAULT,
    ID_REF_MIN, ID_REF_MAX, ID_REF_DEFAULT,
    TELEMETRY_DIV_MIN, TELEMETRY_DIV_MAX, TELEMETRY_DIV_DEFAULT,
    RELAY_COUNT,
)


class SignalEmitter(QObject):
    """Helper class to emit signals from threads."""
    data_received = pyqtSignal(dict)
    status_received = pyqtSignal(dict)
    ack_received = pyqtSignal(int)
    nack_received = pyqtSignal(int, int)
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
        self.signal_emitter.status_received.connect(self._on_status_received)
        self.signal_emitter.ack_received.connect(self._on_ack)
        self.signal_emitter.nack_received.connect(self._on_nack)
        self.signal_emitter.connection_changed.connect(self._on_connection_changed)
        self.signal_emitter.error_occurred.connect(self._on_error)
        
        # Initialize components
        self.serial_comm = SerialCommunicator()
        self.serial_comm.on_data_received = lambda data: self.signal_emitter.data_received.emit(data)
        self.serial_comm.on_status_received = lambda st: self.signal_emitter.status_received.emit(st)
        self.serial_comm.on_ack_received = lambda cmd: self.signal_emitter.ack_received.emit(cmd)
        self.serial_comm.on_nack_received = lambda cmd, err: self.signal_emitter.nack_received.emit(cmd, err)
        self.serial_comm.on_connection_change = lambda status, msg: self.signal_emitter.connection_changed.emit(status, msg)
        self.serial_comm.on_error = lambda msg: self.signal_emitter.error_occurred.emit(msg)
        
        self.data_buffer = TelemetryBuffer()
        
        # CSV logging state
        self._csv_file = None
        self._csv_writer = None
        self._csv_logging = False
        self._csv_sample_count = 0

        # Speed pattern state
        self._pattern_steps = []   # list of (t_seconds, speed_rpm)
        self._pattern_step_idx = 0
        self._pattern_start_wall = 0.0
        self._pattern_running = False

        # Predefined patterns: list of (elapsed_seconds, speed_rpm) tuples
        self.PATTERNS = {
            "Step 0→500→0 RPM": [
                (0.0, 500), (5.0, 0),
            ],
            "Step 0→1000→0 RPM": [
                (0.0, 1000), (8.0, 0),
            ],
            "Multi-Step Up/Down": [
                (0.0,  200), (4.0,  400), (8.0,  700),
                (12.0, 400), (16.0, 200), (20.0,   0),
            ],
            "Hold 700 RPM → Stop": [
                (0.0, 700), (10.0, 0),
            ],
            "Ramp 0→800→0 RPM": (
                [(i * 0.5, i * 100) for i in range(9)] +
                [(4.0 + i * 0.5, 800 - i * 100) for i in range(1, 10)]
            ),
        }
        
        # UI state
        self.connected = False
        self.motor_running = False
        
        # Create UI
        self._create_ui()
        
        # Setup update timer for plots
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(REFRESH_RATE)

        # Pattern runner timer (100 ms tick)
        self._pattern_timer = QTimer()
        self._pattern_timer.timeout.connect(self._pattern_tick)
        self._pattern_timer.setInterval(100)
        
        # Status bar
        self.statusBar().showMessage("Ready. Select a serial port to connect.")
    
    def _create_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        left_panel = self._create_control_panel()
        scroll = QScrollArea()
        scroll.setWidget(left_panel)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(360)
        scroll.setMaximumWidth(440)
        main_layout.addWidget(scroll, 0)

        right_panel = self._create_plot_panel()
        main_layout.addWidget(right_panel, 1)
    
    def _create_control_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        layout.addWidget(self._create_connection_group())
        layout.addWidget(self._create_motor_control_group())
        layout.addWidget(self._create_load_group())
        layout.addWidget(self._create_pattern_group())
        layout.addWidget(self._create_foc_control_group())
        layout.addWidget(self._create_status_group())
        layout.addWidget(self._create_csv_group())
        layout.addWidget(self._create_log_group())
        
        layout.addStretch()
        return panel
    
    def _create_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QFormLayout()
        
        port_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.addItems(self.serial_comm.list_ports())
        refresh_button = QPushButton("Refresh")
        refresh_button.clicked.connect(self._refresh_ports)
        port_layout.addWidget(self.port_combo)
        port_layout.addWidget(refresh_button)
        layout.addRow("Serial Port:", port_layout)
        
        button_layout = QHBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self._on_connect)
        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.clicked.connect(self._on_disconnect)
        self.disconnect_button.setEnabled(False)
        button_layout.addWidget(self.connect_button)
        button_layout.addWidget(self.disconnect_button)
        layout.addRow(button_layout)
        
        self.connection_status_label = QLabel("Disconnected")
        self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
        layout.addRow("Status:", self.connection_status_label)
        
        group.setLayout(layout)
        return group
    
    def _create_motor_control_group(self) -> QGroupBox:
        group = QGroupBox("Motor Control")
        layout = QFormLayout()
        
        # Speed reference (RPM) — firmware converts to rad/s internally
        self.speed_ref_spin = QDoubleSpinBox()
        self.speed_ref_spin.setMinimum(SPEED_REF_MIN)
        self.speed_ref_spin.setMaximum(SPEED_REF_MAX)
        self.speed_ref_spin.setValue(SPEED_REF_DEFAULT)
        self.speed_ref_spin.setSuffix(" RPM")
        self.speed_ref_spin.setSingleStep(10.0)
        self.speed_ref_spin.setDecimals(0)
        speed_apply = QPushButton("Set")
        speed_apply.clicked.connect(self._set_speed_ref)
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(self.speed_ref_spin)
        speed_layout.addWidget(speed_apply)
        layout.addRow("Speed Ref (RPM):", speed_layout)
        
        # Id reference (A)
        self.id_ref_spin = QDoubleSpinBox()
        self.id_ref_spin.setMinimum(ID_REF_MIN)
        self.id_ref_spin.setMaximum(ID_REF_MAX)
        self.id_ref_spin.setValue(ID_REF_DEFAULT)
        self.id_ref_spin.setSuffix(" A")
        self.id_ref_spin.setSingleStep(0.1)
        self.id_ref_spin.setDecimals(3)
        id_apply = QPushButton("Set")
        id_apply.clicked.connect(self._set_id_ref)
        id_layout = QHBoxLayout()
        id_layout.addWidget(self.id_ref_spin)
        id_layout.addWidget(id_apply)
        layout.addRow("Id Ref:", id_layout)
        
        # Telemetry divider
        self.telem_div_spin = QSpinBox()
        self.telem_div_spin.setMinimum(TELEMETRY_DIV_MIN)
        self.telem_div_spin.setMaximum(TELEMETRY_DIV_MAX)
        self.telem_div_spin.setValue(TELEMETRY_DIV_DEFAULT)
        telem_apply = QPushButton("Set")
        telem_apply.clicked.connect(self._set_telemetry_div)
        telem_layout = QHBoxLayout()
        telem_layout.addWidget(self.telem_div_spin)
        telem_layout.addWidget(telem_apply)
        layout.addRow("Telem Divider:", telem_layout)
        
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
        
        # Get Status button
        self.status_button = QPushButton("Get Status")
        self.status_button.clicked.connect(self._get_status)
        self.status_button.setEnabled(False)
        layout.addRow(self.status_button)
        
        group.setLayout(layout)
        return group

    def _create_pattern_group(self) -> QGroupBox:
        group = QGroupBox("Speed Pattern")
        layout = QVBoxLayout()

        # Pattern selector
        self.pattern_combo = QComboBox()
        self.pattern_combo.addItems(list(self.PATTERNS.keys()))
        layout.addWidget(self.pattern_combo)

        # Status label
        self.pattern_status_label = QLabel("Idle")
        self.pattern_status_label.setAlignment(Qt.AlignCenter)
        self.pattern_status_label.setFont(QFont("Courier", 8))
        layout.addWidget(self.pattern_status_label)

        # Start / Stop / Clear History buttons
        btn_layout = QHBoxLayout()
        self.pattern_start_btn = QPushButton("Run Pattern")
        self.pattern_start_btn.setStyleSheet("background-color: #90EE90; font-weight: bold;")
        self.pattern_start_btn.clicked.connect(self._pattern_start)
        self.pattern_stop_btn = QPushButton("Stop")
        self.pattern_stop_btn.setStyleSheet("background-color: #FFB6C1;")
        self.pattern_stop_btn.clicked.connect(self._pattern_stop)
        self.pattern_stop_btn.setEnabled(False)
        self.clear_history_btn = QPushButton("Clear History")
        self.clear_history_btn.clicked.connect(self._clear_history)
        btn_layout.addWidget(self.pattern_start_btn)
        btn_layout.addWidget(self.pattern_stop_btn)
        btn_layout.addWidget(self.clear_history_btn)
        layout.addLayout(btn_layout)

        group.setLayout(layout)
        return group

    def _create_load_group(self) -> QGroupBox:
        group = QGroupBox("Load Control")
        layout = QVBoxLayout()

        # Relay indicator row
        self._load_count = 0
        indicator_layout = QHBoxLayout()
        self._relay_labels = []
        for i in range(RELAY_COUNT):
            lbl = QLabel(f"R{i+1}")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFixedWidth(30)
            lbl.setStyleSheet("background-color: #cccccc; border: 1px solid #888; border-radius: 3px;")
            indicator_layout.addWidget(lbl)
            self._relay_labels.append(lbl)
        layout.addLayout(indicator_layout)

        # +/- buttons and remove-all
        btn_layout = QHBoxLayout()
        self.load_dec_btn = QPushButton("− Relay")
        self.load_dec_btn.clicked.connect(self._load_decrease)
        self.load_inc_btn = QPushButton("+ Relay")
        self.load_inc_btn.clicked.connect(self._load_increase)
        self.load_clear_btn = QPushButton("No Load")
        self.load_clear_btn.clicked.connect(self._load_clear)
        self.load_all_btn = QPushButton("Full Load")
        self.load_all_btn.clicked.connect(self._load_all)
        btn_layout.addWidget(self.load_dec_btn)
        btn_layout.addWidget(self.load_inc_btn)
        btn_layout.addWidget(self.load_clear_btn)
        btn_layout.addWidget(self.load_all_btn)
        layout.addLayout(btn_layout)

        self.load_count_label = QLabel("Active relays: 0")
        self.load_count_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.load_count_label)

        group.setLayout(layout)
        return group

    def _create_foc_control_group(self) -> QGroupBox:
        group = QGroupBox("PI Tuning")
        layout = QFormLayout()

        # Current PI (shared d+q axes)
        self.cur_kp_spin = QDoubleSpinBox()
        self.cur_ki_spin = QDoubleSpinBox()
        for spin in (self.cur_kp_spin, self.cur_ki_spin):
            spin.setRange(0.0, 1000.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.1)
        cur_pi_layout = QHBoxLayout()
        cur_pi_layout.addWidget(QLabel("Kp"))
        cur_pi_layout.addWidget(self.cur_kp_spin)
        cur_pi_layout.addWidget(QLabel("Ki"))
        cur_pi_layout.addWidget(self.cur_ki_spin)
        cur_pi_button = QPushButton("Set")
        cur_pi_button.clicked.connect(self._set_current_pi)
        cur_pi_layout.addWidget(cur_pi_button)
        layout.addRow("Current PI:", cur_pi_layout)

        # Speed PI
        self.spd_kp_spin = QDoubleSpinBox()
        self.spd_ki_spin = QDoubleSpinBox()
        for spin in (self.spd_kp_spin, self.spd_ki_spin):
            spin.setRange(0.0, 1000.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.1)
        spd_pi_layout = QHBoxLayout()
        spd_pi_layout.addWidget(QLabel("Kp"))
        spd_pi_layout.addWidget(self.spd_kp_spin)
        spd_pi_layout.addWidget(QLabel("Ki"))
        spd_pi_layout.addWidget(self.spd_ki_spin)
        spd_pi_button = QPushButton("Set")
        spd_pi_button.clicked.connect(self._set_speed_pi)
        spd_pi_layout.addWidget(spd_pi_button)
        layout.addRow("Speed PI:", spd_pi_layout)

        group.setLayout(layout)
        return group
    
    def _create_status_group(self) -> QGroupBox:
        group = QGroupBox("Live Telemetry")
        layout = QFormLayout()
        
        self.speed_label = QLabel("0.0 RPM")
        self.speed_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addRow("Speed (omega_m):", self.speed_label)
        
        self.id_label = QLabel("Id: 0.000 A")
        self.id_label.setFont(QFont("Arial", 10))
        layout.addRow("d-axis Current:", self.id_label)
        
        self.iq_label = QLabel("Iq: 0.000 A")
        self.iq_label.setFont(QFont("Arial", 10))
        layout.addRow("q-axis Current:", self.iq_label)
        
        self.vbus_label = QLabel("0.00 V")
        self.vbus_label.setFont(QFont("Arial", 10))
        layout.addRow("DC Bus Voltage:", self.vbus_label)
        
        self.phase_current_label = QLabel("Ia: 0.000, Ib: 0.000, Ic: 0.000 A")
        self.phase_current_label.setFont(QFont("Arial", 10))
        layout.addRow("Phase Currents:", self.phase_current_label)
        
        # MCU status (from GET_STATUS response)
        self.mcu_state_label = QLabel("--")
        self.mcu_state_label.setFont(QFont("Arial", 10))
        layout.addRow("Motor State:", self.mcu_state_label)
        
        self.mcu_uptime_label = QLabel("--")
        self.mcu_uptime_label.setFont(QFont("Arial", 10))
        layout.addRow("MCU Uptime:", self.mcu_uptime_label)
        
        group.setLayout(layout)
        return group
    
    def _create_csv_group(self) -> QGroupBox:
        group = QGroupBox("NN Data Logger")
        layout = QVBoxLayout()

        # File path row
        path_layout = QHBoxLayout()
        self.csv_path_edit = QLineEdit("nn_training_data.csv")
        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self._csv_browse)
        path_layout.addWidget(self.csv_path_edit)
        path_layout.addWidget(browse_btn)
        layout.addLayout(path_layout)

        # Start/Stop and sample counter
        ctrl_layout = QHBoxLayout()
        self.csv_start_btn = QPushButton("Start Logging")
        self.csv_start_btn.clicked.connect(self._csv_start)
        self.csv_stop_btn = QPushButton("Stop Logging")
        self.csv_stop_btn.clicked.connect(self._csv_stop)
        self.csv_stop_btn.setEnabled(False)
        self.csv_count_label = QLabel("0 samples")
        ctrl_layout.addWidget(self.csv_start_btn)
        ctrl_layout.addWidget(self.csv_stop_btn)
        ctrl_layout.addWidget(self.csv_count_label)
        layout.addLayout(ctrl_layout)

        group.setLayout(layout)
        return group

    def _csv_browse(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save CSV", self.csv_path_edit.text(),
            "CSV Files (*.csv);;All Files (*)"
        )
        if path:
            self.csv_path_edit.setText(path)

    def _csv_start(self):
        path = self.csv_path_edit.text().strip()
        if not path:
            self._add_log_message("ERROR: No CSV file path specified")
            return
        try:
            self._csv_file = open(path, 'w', newline='', encoding='utf-8')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(['iq', 'omega_m', 'omega_m_ref', 'imr', 'dwr_dt'])
            self._csv_logging = True
            self._csv_sample_count = 0
            self.csv_start_btn.setEnabled(False)
            self.csv_stop_btn.setEnabled(True)
            self._add_log_message(f"CSV logging started: {path}")
        except Exception as e:
            self._add_log_message(f"ERROR: Cannot open CSV file: {e}")

    def _csv_stop(self):
        self._csv_logging = False
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        self.csv_start_btn.setEnabled(True)
        self.csv_stop_btn.setEnabled(False)
        self._add_log_message(f"CSV logging stopped. {self._csv_sample_count} samples saved.")

    def _create_log_group(self) -> QGroupBox:
        group = QGroupBox("Messages")
        layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setFont(QFont("Courier", 8))
        layout.addWidget(self.log_text)
        group.setLayout(layout)
        return group
    
    def _make_plot_tab(self, tabs: QTabWidget, title: str):
        """Create a canvas+toolbar container and add it as a tab. Returns (canvas, axes)."""
        canvas = FigureCanvas(Figure(figsize=(8, 3)))
        ax = canvas.figure.add_subplot(111)
        toolbar = NavigationToolbar(canvas, self)
        container = QWidget()
        vbox = QVBoxLayout(container)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)
        vbox.addWidget(toolbar)
        vbox.addWidget(canvas)
        tabs.addTab(container, title)
        return canvas, ax

    def _create_plot_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        tabs = QTabWidget()
        
        self.speed_canvas,   self.speed_ax   = self._make_plot_tab(tabs, "Speed (RPM)")
        self.park_canvas,    self.park_ax    = self._make_plot_tab(tabs, "Park Currents (A)")
        self.current_canvas, self.current_ax = self._make_plot_tab(tabs, "Phase Currents (A)")
        self.vbus_canvas,    self.vbus_ax    = self._make_plot_tab(tabs, "Bus Voltage (V)")
        self.theta_canvas,   self.theta_ax   = self._make_plot_tab(tabs, "Theta_e (rad)")
        self.torque_canvas,  self.torque_ax  = self._make_plot_tab(tabs, "Torque (N·m)")
        
        layout.addWidget(tabs)
        return panel
    
    # ---- Connection Methods ----
    
    def _refresh_ports(self):
        self.port_combo.clear()
        ports = self.serial_comm.list_ports()
        self.port_combo.addItems(ports)
        if ports:
            self._add_log_message(f"Found {len(ports)} port(s): {', '.join(ports)}")
        else:
            self._add_log_message("No serial ports found")
    
    def _on_connect(self):
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
            self.status_button.setEnabled(True)
            self._add_log_message(f"Connected to {port} @ 230400 baud")
        else:
            self._add_log_message(f"Failed to connect to {port}")
    
    def _on_disconnect(self):
        self.serial_comm.disconnect()
        self.connected = False
        self.motor_running = False
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.status_button.setEnabled(False)
        self._add_log_message("Disconnected")
    
    def _on_connection_changed(self, status: bool, message: str):
        if status:
            self.connection_status_label.setText("Connected")
            self.connection_status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.connection_status_label.setText("Disconnected")
            self.connection_status_label.setStyleSheet("color: red; font-weight: bold;")
        self._add_log_message(message)
    
    # ---- Command Methods ----
    
    def _set_speed_ref(self):
        val = max(0.0, self.speed_ref_spin.value())
        self.speed_ref_spin.setValue(val)
        if self.serial_comm.set_speed_ref(val):
            self.data_buffer.set_speed_reference(val)
            self._add_log_message(f"Speed ref set to {val:.0f} RPM")
        else:
            self._add_log_message("ERROR: Failed to set speed ref")
    
    def _set_id_ref(self):
        val = self.id_ref_spin.value()
        if self.serial_comm.set_id_ref(val):
            self._add_log_message(f"Id ref set to {val:.3f} A")
        else:
            self._add_log_message("ERROR: Failed to set Id ref")
    
    def _set_telemetry_div(self):
        val = self.telem_div_spin.value()
        if self.serial_comm.set_telemetry_div(val):
            self.data_buffer.set_telemetry_divider(val)
            msg = f"Telemetry divider set to {val}" if val > 0 else "Telemetry disabled"
            self._add_log_message(msg)
        else:
            self._add_log_message("ERROR: Failed to set telemetry divider")
    
    def _set_current_pi(self):
        kp = self.cur_kp_spin.value()
        ki = self.cur_ki_spin.value()
        if self.serial_comm.set_current_pi(kp, ki):
            self._add_log_message(f"Current PI set: Kp={kp}, Ki={ki}")
        else:
            self._add_log_message("ERROR: Failed to set current PI")
    
    def _set_speed_pi(self):
        kp = self.spd_kp_spin.value()
        ki = self.spd_ki_spin.value()
        if self.serial_comm.set_speed_pi(kp, ki):
            self._add_log_message(f"Speed PI set: Kp={kp}, Ki={ki}")
        else:
            self._add_log_message("ERROR: Failed to set speed PI")
    
    def _start_motor(self):
        if self.serial_comm.start_motor():
            self.motor_running = True
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self._add_log_message("Motor START sent")
        else:
            self._add_log_message("ERROR: Failed to send motor start")
    
    def _stop_motor(self):
        if self.serial_comm.stop_motor():
            self.motor_running = False
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self._add_log_message("Motor STOP sent")
        else:
            self._add_log_message("ERROR: Failed to send motor stop")
    
    def _get_status(self):
        if self.serial_comm.get_status():
            self._add_log_message("Status request sent")
        else:
            self._add_log_message("ERROR: Failed to request status")

    # ---- Load control ----

    def _apply_load(self, count: int):
        self._load_count = max(0, min(RELAY_COUNT, count))
        if self.serial_comm.set_load(self._load_count):
            self._add_log_message(f"Load set to {self._load_count} relay(s)")
        else:
            self._add_log_message("ERROR: Failed to set load")
        # Update indicators
        for i, lbl in enumerate(self._relay_labels):
            if i < self._load_count:
                lbl.setStyleSheet("background-color: #ff6b35; border: 1px solid #c0392b; border-radius: 3px; color: white; font-weight: bold;")
            else:
                lbl.setStyleSheet("background-color: #cccccc; border: 1px solid #888; border-radius: 3px;")
        self.load_count_label.setText(f"Active relays: {self._load_count} / {RELAY_COUNT}")

    def _load_increase(self):
        self._apply_load(self._load_count + 1)

    def _load_decrease(self):
        self._apply_load(self._load_count - 1)

    def _load_clear(self):
        self._apply_load(0)

    def _load_all(self):
        self._apply_load(RELAY_COUNT)

    # ---- Pattern runner ----

    def _pattern_start(self):
        name = self.pattern_combo.currentText()
        self._pattern_steps = list(self.PATTERNS[name])
        self._pattern_step_idx = 0
        self._pattern_start_wall = time.time()
        self._pattern_running = True
        self.pattern_start_btn.setEnabled(False)
        self.pattern_stop_btn.setEnabled(True)
        self._pattern_timer.start()
        total = self._pattern_steps[-1][0]
        self._add_log_message(f"Pattern '{name}' started ({len(self._pattern_steps)} steps, {total:.1f}s)")

    def _pattern_stop(self):
        self._pattern_running = False
        self._pattern_timer.stop()
        self.pattern_start_btn.setEnabled(True)
        self.pattern_stop_btn.setEnabled(False)
        self.pattern_status_label.setText("Stopped")
        self._add_log_message("Pattern stopped")

    def _pattern_tick(self):
        if not self._pattern_running:
            return
        elapsed = time.time() - self._pattern_start_wall

        # Dispatch any steps whose time has come
        while (self._pattern_step_idx < len(self._pattern_steps) and
               elapsed >= self._pattern_steps[self._pattern_step_idx][0]):
            t, rpm = self._pattern_steps[self._pattern_step_idx]
            self.serial_comm.set_speed_ref(rpm)
            self.data_buffer.set_speed_reference(rpm)
            self._pattern_step_idx += 1

        # Update status label
        total = self._pattern_steps[-1][0]
        if self._pattern_step_idx < len(self._pattern_steps):
            next_t, next_rpm = self._pattern_steps[self._pattern_step_idx]
            cur_rpm = self._pattern_steps[self._pattern_step_idx - 1][1] if self._pattern_step_idx > 0 else 0
            remaining = next_t - elapsed
            self.pattern_status_label.setText(
                f"{elapsed:.1f}s / {total:.1f}s | {cur_rpm:+.0f} RPM → {next_rpm:+.0f} RPM in {remaining:.1f}s"
            )
        else:
            self.pattern_status_label.setText(f"Done ({total:.1f}s)")
            self._pattern_stop()

    def _clear_history(self):
        self.data_buffer.clear()
        # Reset zoom state on all axes so they autoscale again
        for ax in (self.speed_ax, self.park_ax, self.current_ax,
                   self.vbus_ax, self.theta_ax, self.torque_ax):
            ax.set_autoscalex_on(True)
            ax.set_autoscaley_on(True)
        self._add_log_message("Plot history cleared")

    # ---- Response Handlers ----
    
    def _on_ack(self, cmd_echo: int):
        self._add_log_message(f"ACK for cmd 0x{cmd_echo:02X}")
    
    def _on_nack(self, cmd_echo: int, error_code: int):
        err_str = NACK_ERRORS.get(error_code, f"0x{error_code:02X}")
        self._add_log_message(f"NACK for cmd 0x{cmd_echo:02X}: {err_str}")
    
    def _on_data_received(self, data: dict):
        self.data_buffer.add_telemetry(
            data['id'], data['iq'], data['vbus'],
            data['omega_m'], data['ia'], data['ib'], data['ic'],
            data.get('theta_e', 0.0), data.get('torque_e', 0.0),
            data.get('imr', 0.0), data.get('dwr_dt', 0.0),
        )
        # Write to CSV if logging — omega_m_ref converted from RPM to rad/s
        if self._csv_logging and self._csv_writer is not None:
            import math
            omega_m_ref_rads = self.data_buffer.reference_speed * math.pi / 30.0
            self._csv_writer.writerow([
                f"{data['iq']:.6f}",
                f"{data['omega_m']:.6f}",
                f"{omega_m_ref_rads:.6f}",
                f"{data.get('imr', 0.0):.6f}",
                f"{data.get('dwr_dt', 0.0):.6f}",
            ])
            self._csv_sample_count += 1
            self.csv_count_label.setText(f"{self._csv_sample_count} samples")
        self._update_status_display(data)
    
    def _on_status_received(self, status: dict):
        self.mcu_state_label.setText(status['motor_state_str'])
        self.mcu_uptime_label.setText(f"{status['uptime_s']} s")
        self._add_log_message(
            f"STATUS: {status['motor_state_str']}, "
            f"omega_ref={status['omega_ref']:.2f} rad/s, "
            f"id_ref={status['id_ref']:.3f} A, "
            f"faults=0x{status['fault_flags']:02X}, "
            f"uptime={status['uptime_s']}s"
        )
    
    def _update_status_display(self, data: dict):
        latest = self.data_buffer.get_latest_values()
        omega_rpm = latest['omega_m'] * 60.0 / (2.0 * 3.141592653589793)
        self.speed_label.setText(f"{omega_rpm:.1f} RPM")
        self.id_label.setText(f"Id: {latest['id']:.3f} A")
        self.iq_label.setText(f"Iq: {latest['iq']:.3f} A")
        self.vbus_label.setText(f"{latest['vbus']:.2f} V")
        self.phase_current_label.setText(
            f"Ia: {latest['ia']:.3f}, Ib: {latest['ib']:.3f}, Ic: {latest['ic']:.3f} A"
        )
    
    # ---- Plot Methods ----
    
    def _update_plots(self):
        plot_data = self.data_buffer.get_plot_data()
        if len(plot_data['time']) > 0:
            self._plot_speed(plot_data)
            self._plot_park(plot_data)
            self._plot_current(plot_data)
            self._plot_vbus(plot_data)
            self._plot_theta_e(plot_data)
            self._plot_torque(plot_data)
    
    @staticmethod
    def _save_view(ax):
        """Return current axis limits if the user has zoomed/panned, else None."""
        if not ax.get_autoscalex_on() or not ax.get_autoscaley_on():
            return ax.get_xlim(), ax.get_ylim()
        return None

    @staticmethod
    def _restore_view(ax, view):
        """Re-apply saved axis limits and disable autoscale to keep the view locked."""
        if view is not None:
            xlim, ylim = view
            ax.set_xlim(xlim)
            ax.set_ylim(ylim)
            ax.set_autoscalex_on(False)
            ax.set_autoscaley_on(False)

    def _plot_speed(self, data: dict):
        view = self._save_view(self.speed_ax)
        self.speed_ax.clear()
        t = data['time']
        omega_rpm = data['omega_m'] * 60.0 / (2.0 * 3.141592653589793)
        self.speed_ax.plot(t, omega_rpm, 'b-', label='Actual', linewidth=1.5)
        self.speed_ax.plot(t, data['speed_reference'], 'r--', label='Reference', linewidth=1.5)
        self.speed_ax.set_xlabel('Time (s)')
        self.speed_ax.set_ylabel('Speed (RPM)')
        self.speed_ax.set_title('Mechanical Speed')
        self.speed_ax.legend(loc='best')
        self.speed_ax.grid(True, alpha=0.3)
        self._restore_view(self.speed_ax, view)
        self.speed_canvas.draw()
    
    def _plot_park(self, data: dict):
        view = self._save_view(self.park_ax)
        self.park_ax.clear()
        t = data['time']
        self.park_ax.plot(t, data['id'], 'g-', label='Id', linewidth=1.5)
        self.park_ax.plot(t, data['iq'], 'm-', label='Iq', linewidth=1.5)
        self.park_ax.set_xlabel('Time (s)')
        self.park_ax.set_ylabel('Current (A)')
        self.park_ax.set_title('Park Currents (d-q)')
        self.park_ax.legend(loc='best')
        self.park_ax.grid(True, alpha=0.3)
        self._restore_view(self.park_ax, view)
        self.park_canvas.draw()
    
    def _plot_current(self, data: dict):
        view = self._save_view(self.current_ax)
        self.current_ax.clear()
        t = data['time']
        self.current_ax.plot(t, data['ia'], 'c-', label='Ia', linewidth=1.5)
        self.current_ax.plot(t, data['ib'], 'y-', label='Ib', linewidth=1.5)
        self.current_ax.plot(t, data['ic'], 'k-', label='Ic', linewidth=1.5)
        self.current_ax.set_xlabel('Time (s)')
        self.current_ax.set_ylabel('Current (A)')
        self.current_ax.set_title('Phase Currents')
        self.current_ax.legend(loc='best')
        self.current_ax.grid(True, alpha=0.3)
        self._restore_view(self.current_ax, view)
        self.current_canvas.draw()
    
    def _plot_vbus(self, data: dict):
        view = self._save_view(self.vbus_ax)
        self.vbus_ax.clear()
        t = data['time']
        self.vbus_ax.plot(t, data['vbus'], 'r-', label='Vbus', linewidth=1.5)
        self.vbus_ax.set_xlabel('Time (s)')
        self.vbus_ax.set_ylabel('Voltage (V)')
        self.vbus_ax.set_title('DC Bus Voltage')
        self.vbus_ax.legend(loc='best')
        self.vbus_ax.grid(True, alpha=0.3)
        self._restore_view(self.vbus_ax, view)
        self.vbus_canvas.draw()
    
    def _plot_theta_e(self, data: dict):
        view = self._save_view(self.theta_ax)
        self.theta_ax.clear()
        t = data['time']
        self.theta_ax.plot(t, data['theta_e'], 'b-', label='theta_e', linewidth=1.5)
        self.theta_ax.set_xlabel('Time (s)')
        self.theta_ax.set_ylabel('Angle (rad)')
        self.theta_ax.set_title('Electrical Angle (theta_e)')
        self.theta_ax.legend(loc='best')
        self.theta_ax.grid(True, alpha=0.3)
        self._restore_view(self.theta_ax, view)
        self.theta_canvas.draw()

    def _plot_torque(self, data: dict):
        view = self._save_view(self.torque_ax)
        self.torque_ax.clear()
        t = data['time']
        self.torque_ax.plot(t, data['torque_e'], 'darkorange', label='torque_e', linewidth=1.5)
        self.torque_ax.set_xlabel('Time (s)')
        self.torque_ax.set_ylabel('Torque (N·m)')
        self.torque_ax.set_title('Estimated Electromagnetic Torque')
        self.torque_ax.legend(loc='best')
        self.torque_ax.grid(True, alpha=0.3)
        self._restore_view(self.torque_ax, view)
        self.torque_canvas.draw()
    
    # ---- Utility ----
    
    def _on_error(self, message: str):
        self._add_log_message(f"ERR: {message}")
    
    def _add_log_message(self, message: str):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def closeEvent(self, event):
        if self._pattern_running:
            self._pattern_stop()
        if self._csv_logging:
            self._csv_stop()
        if self.connected:
            self.serial_comm.disconnect()
        event.accept()
