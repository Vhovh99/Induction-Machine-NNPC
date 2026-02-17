# Technical Documentation - Motor Control UI

## Architecture Overview

The Motor Control UI follows a modular, event-driven architecture:

```
┌─────────────────────────────────────────────────────┐
│              PyQt5 Main Window (UI)                 │
│        - Handles user interactions                  │
│        - Manages plot rendering                     │
│        - Updates status displays                    │
└────────────────┬────────────────────────────────────┘
                 │
    ┌────────────┴──────────────┐
    │                           │
┌───▼──────────────────┐  ┌────▼─────────────────┐
│ Serial Communicator  │  │   Data Buffer       │
│ - Thread-safe        │  │ - Circular buffers  │
│ - Command sending    │  │ - Torque calcs      │
│ - Data parsing       │  │ - Data smoothing    │
└────────────┬─────────┘  └────┬────────────────┘
             │                 │
             └─────────┬───────┘
                       │
               ┌───────▼──────────┐
               │  Serial Port     │
               │  (115200 baud)   │
               └──────────────────┘
```

## Module Descriptions

### `main.py`
- **Purpose**: Application entry point
- **Responsibilities**:
  - Create QApplication instance
  - Initialize main window
  - Start event loop

### `app_window.py`
- **Purpose**: Main GUI window and user interface
- **Key Classes**:
  - `MotorControlUI`: Main PyQt5 window
  - `SignalEmitter`: Thread-safe signal bridge
- **Responsibilities**:
  - UI layout and widgets
  - Plot management
  - Command handling
  - Status display updates
- **Key Methods**:
  - `_create_ui()`: Build UI layout
  - `_update_plots()`: Refresh visualization
  - `_on_data_received()`: Handle new telemetry

### `serial_communicator.py`
- **Purpose**: Handle all serial port communication
- **Key Class**: `SerialCommunicator`
- **Responsibilities**:
  - Open/close serial connections
  - Send motor control commands
  - Parse incoming telemetry
  - Background receive thread
- **Thread Safety**: Uses queue and callback system
- **Key Methods**:
  - `connect(port)`: Establish connection
  - `send_command(cmd)`: Send ASCII command
  - `_receive_loop()`: Background thread loop
  - `_parse_telemetry(line)`: Parse "SPD:xxx CUR:x,y" format

### `data_buffer.py`
- **Purpose**: Store telemetry data and perform calculations
- **Key Class**: `TelemetryBuffer`
- **Responsibilities**:
  - Maintain circular buffers for all data
  - Calculate derived values (torque, etc.)
  - Provide data for plotting
- **Key Methods**:
  - `add_telemetry()`: Add new sample
  - `get_plot_data()`: Return arrays for plotting
  - `_calculate_applied_torque()`: T = K * I
  - `_calculate_electromagnetic_torque()`: T_em = J*dω/dt + T_friction

### `config.py`
- **Purpose**: Centralized configuration
- **Contains**:
  - Serial communication settings
  - UI sizing parameters
  - Motor model constants
  - Control limits
  - Calibration values

## Communication Protocol

### Command Format (Host → Motor)
```
F <frequency>     Set frequency (Hz)
A <amplitude>     Set amplitude (0.0-1.0)
R <rpm>           Set target RPM
S                 Start motor
X                 Stop motor
```
All commands end with newline character.

### Telemetry Format (Motor → Host)
```
SPD:<speed> CUR:<ia>,<ib>
```
- **SPD**: Speed in RPM (float)
- **CUR**: Phase currents in Amperes (floats, comma-separated)
- Received every 100ms

## Data Flow

### Command Sending
```
User clicks button
  → app_window.py calls serial_comm method
    → serial_communicator.py formats command
      → Sends via serial.write()
        → Motor controller receives
```

### Telemetry Reception
```
Motor controller sends: "SPD:1500.0 CUR:0.45,-0.42\n"
  → serial_communicator.py reads bytes
    → _receive_loop() parses telemetry
      → Emits signal to main thread
        → data_buffer.py stores sample
          → Calculations run (torque, etc.)
            → Plot timer triggers update
              → Plots refresh
```

### Threading Model
```
Main Thread (UI)
  ├─ PyQt5 Event Loop
  ├─ Plot Update Timer (50ms)
  └─ Signal handling

Serial Receive Thread
  └─ Continuous loop
     ├─ Read from serial port
     ├─ Parse data
     └─ Add to queue
```

## Torque Calculation Details

### Applied Torque
```python
T_applied = K_motor * sqrt(Ia² + Ib²)
```
Where:
- K_motor = motor constant (default: 2.5 N⋅m/A)
- Ia, Ib = phase currents

**Calibration**: Measure actual torque at known current, adjust K_motor

### Electromagnetic Torque
Uses motor dynamic equation:
```python
T_em = J * (dω/dt) + T_friction
T_friction = C_friction * ω
```

Where:
- J = moment of inertia (kg⋅m²)
- ω = angular velocity (rad/s)
- C_friction = friction coefficient

**Process**:
1. Measure speed change over time dt
2. Calculate angular acceleration: dω/dt
3. Calculate friction torque: proportional to speed
4. Sum to get electromagnetic torque

## Circular Buffer Implementation

The `TelemetryBuffer` uses Python's `deque` with `maxlen`:
```python
self.speeds = deque(maxlen=BUFFER_SIZE)  # Auto-removes old entries
```

**Advantages**:
- Automatic size limiting
- O(1) append and read operations
- Memory efficient (no copying)
- Suitable for real-time data

**Buffer Size**: 500 samples
- At 10 Hz telemetry: ~50 seconds of data
- Adjustable in `config.py`

## Extension Points

### Adding New Plots
In `app_window.py`:
```python
# 1. Create canvas in _create_plot_panel()
self.new_canvas = FigureCanvas(Figure(figsize=(8, 3)))
tabs.addTab(self.new_canvas, "New Plot")

# 2. Create axes
self.new_ax = self.new_canvas.figure.add_subplot(111)

# 3. Add plot method
def _plot_new_data(self, data):
    self.new_ax.clear()
    self.new_ax.plot(data['time'], data['something'])
    self.new_canvas.draw()

# 4. Call from _update_plots()
self._plot_new_data(plot_data)
```

### Adding New Telemetry
In `serial_communicator.py`:
```python
# In _parse_telemetry():
if 'NEW:' in line:
    new_start = line.index('NEW:') + 4
    data['new_value'] = float(line[new_start:].split()[0])
```

In `app_window.py`, update status display.

### Adding Motor Constants
In `config.py`:
```python
MOTOR_CONSTANT_NEW = 3.5
```

In `data_buffer.py`:
```python
from config import MOTOR_CONSTANT_NEW

def _calculate_new_value(self):
    return MOTOR_CONSTANT_NEW * something
```

### Data Logging
Add to `app_window.py`:
```python
import csv
from datetime import datetime

def _log_to_csv(self):
    filename = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time', 'Speed', 'Torque', 'Current'])
        for t, s, tor, cur in zip(...):
            writer.writerow([t, s, tor, cur])
```

## Performance Considerations

### CPU Usage
- UI plotting: ~1-2%
- Serial communication: <0.5%
- Total: ~2-3% on modern CPU

### Memory Usage
- With 500-sample buffers: ~20 KB
- UI widgets: ~50 MB (PyQt5)
- Matplotlib: ~30 MB
- Total: ~100 MB

### Real-Time Requirements
- Serial communication: 100ms (non-blocking thread)
- Plot updates: 50ms (acceptable for humans)
- UI responsiveness: <100ms (not affected by plotting)

## Known Limitations

1. **Buffer Size**: 500 samples limits recording to ~50 seconds at 10 Hz
   - Solution: Write to disk periodically

2. **Torque Estimation**: Simplified model doesn't account for:
   - Motor saturation
   - Temperature effects
   - Advanced loss models
   - Solution: Implement advanced models in `data_buffer.py`

3. **Single Motor**: UI designed for one controller
   - Solution: Use tab widget for multiple motors

4. **No Data Validation**: Assumes correct telemetry format
   - Solution: Add error checking in `_parse_telemetry()`

## Debugging

### Enable Verbose Logging
Add to main.py:
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Serial Port Debugging
Check actual data:
```python
# In serial_communicator.py _receive_loop()
print(f"RAW: {repr(line)}")  # Shows exact bytes
```

### Plot Debugging
```python
# In app_window.py _plot_speed()
print(f"Data points: {len(data['time'])}")
print(f"Speed range: {data['speed_actual'].min()}-{data['speed_actual'].max()}")
```

## Future Improvements

### High Priority
- [ ] Configuration profile save/load
- [ ] Data export to CSV with timestamp
- [ ] Error correction/validation for telemetry
- [ ] Tuning parameter autodetection

### Medium Priority
- [ ] FFT analysis of currents
- [ ] Advanced torque models (temperature-compensated)
- [ ] PID loop tuning visualization
- [ ] Multi-motor support

### Low Priority
- [ ] 3D plots for motor dynamics
- [ ] Machine learning-based anomaly detection
- [ ] Mobile app control
- [ ] Cloud data logging

---

For questions about specific implementations, see the source code comments.
