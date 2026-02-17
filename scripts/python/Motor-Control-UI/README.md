# Motor Control UI - Induction Machine FOC

A comprehensive Python GUI application for controlling an induction motor with Field-Oriented Control (FOC). The application provides real-time monitoring and control of motor parameters through serial communication.

## Features

- **Real-time Monitoring**
  - Actual motor speed (RPM)
  - Speed reference tracking
  - Phase currents (Ia, Ib)
  - Applied torque calculation
  - Electromagnetic torque estimation

- **Interactive Control**
  - Set inverter frequency (0-100 Hz)
  - Set inverter amplitude (0.0-1.0)
  - Set target RPM
  - Start/Stop motor commands

- **Data Visualization**
  - Speed vs Reference Speed plot
  - Applied vs Electromagnetic Torque plot
  - Phase current waveforms

- **Serial Communication**
  - Configurable serial ports
  - 115200 baud rate
  - Automatic telemetry parsing
  - Real-time data buffering

## System Requirements

- Python 3.7 or higher
- PyQt5 5.15+
- matplotlib 3.8+
- numpy 1.26+
- pyserial 3.5+

## Installation

### 1. Clone or Download the Project

```bash
cd Motor-Control-UI
```

### 2. Create a Virtual Environment (Recommended)

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

## Usage

### Starting the Application

```bash
python main.py
```

### Connection Setup

1. **Select Serial Port**: Choose the COM port connected to your motor controller
2. **Click "Connect"**: Establish serial connection
3. The status indicator should turn green when connected

### Motor Control

#### Manual Frequency Control
- Set desired frequency (Hz) in the "Frequency" field
- Click "Set" button
- Command sent: `F <frequency>`

#### Amplitude Control
- Set inverter output amplitude (0.0 to 1.0) in the "Amplitude" field
- Click "Set" button
- Command sent: `A <amplitude>`

#### RPM Control
- Set target speed in the "Target RPM" field
- Click "Set" button
- Command sent: `R <rpm>`

#### Motor Operation
- **Start Motor**: Click "Start Motor" button (sends `S` command)
- **Stop Motor**: Click "Stop Motor" button (sends `X` command)

### Monitoring

The application displays:
- **Current Status**: Real-time speed, phase currents, and torque values
- **Speed Plot**: Compares actual and reference speeds over time
- **Torque Plot**: Shows applied and electromagnetic torques
- **Current Plot**: Phase current waveforms (Ia, Ib)
- **Message Log**: All communications and status messages

## Communication Protocol

### Commands Sent to Motor Controller

```
F <value>     : Set Frequency in Hz
                Example: F 50.0

A <value>     : Set Amplitude (0.0 - 1.0)
                Example: A 0.5

R <value>     : Set RPM
                Example: R 1500

S             : Start Motor

X             : Stop Motor
```

### Telemetry Received

The motor controller sends telemetry data every 100ms:

```
SPD:<speed> CUR:<ia>,<ib>
Example: SPD:1495.20 CUR:0.45,-0.42
```

- **SPD**: Current motor speed in RPM
- **CUR**: Phase currents Ia and Ib in Amperes

## Configuration

Edit `config.py` to customize:

```python
# Serial Communication
SERIAL_BAUDRATE = 115200
TELEMETRY_INTERVAL = 100  # milliseconds

# Data Buffer Size
BUFFER_SIZE = 500  # samples to keep in memory

# Motor Parameters
MOTOR_POLE_PAIRS = 2
MOTOR_MOMENT_OF_INERTIA = 0.001  # kg*m^2
FRICTION_COEFFICIENT = 0.01  # N*m*s

# Control Limits
FREQUENCY_MIN = 0.0
FREQUENCY_MAX = 100.0
FREQUENCY_DEFAULT = 50.0

AMPLITUDE_MIN = 0.0
AMPLITUDE_MAX = 1.0
AMPLITUDE_DEFAULT = 0.5

RPM_MIN = 0
RPM_MAX = 3000
RPM_DEFAULT = 1500
```

## Torque Calculation

The application calculates two torque values:

### Applied Torque
```
T_applied = K_motor * sqrt(Ia^2 + Ib^2)
```
where K_motor is a calibration constant (default: 2.5 N*m/A)

### Electromagnetic Torque
Estimated from the dynamic equation:
```
T_em = J * dω/dt + T_friction
```
where:
- J = moment of inertia
- ω = angular velocity
- T_friction = FRICTION_COEFFICIENT * speed

## File Structure

```
Motor-Control-UI/
├── main.py                 # Application entry point
├── app_window.py          # Main PyQt5 window and UI
├── serial_communicator.py  # Serial communication handler
├── data_buffer.py         # Telemetry storage and calculations
├── config.py              # Configuration settings
├── requirements.txt       # Python dependencies
└── README.md              # This file
```

## Troubleshooting

### Serial Port Not Found
- Ensure the motor controller is connected and powered on
- Check if the correct COM port is selected
- Click "Refresh" to rescan available ports
- On Linux, you may need to run: `sudo chmod 666 /dev/ttyUSB0`

### Connection Failed
- Verify the serial cable is properly connected
- Check that the motor controller is operating at 115200 baud rate
- Try disconnecting and reconnecting the USB adapter

### No Data Received
- Ensure the motor controller is properly initialized
- Check the message log for any errors
- Verify the telemetry format matches: `SPD:xxx CUR:x,y`

### Plots Not Updating
- Motor must be running to generate telemetry data
- Check that serial communication is working (see previous sections)
- Verify the "Current Status" displays updated speed and current values

## Performance Notes

- The application buffers up to 500 samples for each parameter
- Plot refresh rate: 50ms (20 Hz)
- Serial telemetry interval: 100ms (10 Hz)
- CPU usage is typically <2% on modern systems

## Future Enhancements

Possible additions:
- Data logging to CSV files
- Export plots as images
- Real-time FFT analysis of currents
- Advanced torque estimation models
- Calibration wizard for motor constants
- Support for multiple motor controllers
- Configuration profiles

## License

This project is part of the Induction Machine NNPC control system.

## Support

For issues or questions, please check:
1. The message log in the application
2. The serial connection status
3. The communication protocol documentation above
4. The configuration settings in `config.py`
