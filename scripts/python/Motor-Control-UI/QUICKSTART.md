# Quick Start Guide - Motor Control UI

## 5-Minute Setup

### Step 1: Install Dependencies

**Linux/Mac:**
```bash
bash setup.sh
```

**Windows:**
```bash
setup.bat
```

### Step 2: Connect Your Motor Controller

- Plug in the motor controller via USB/Serial cable
- Note the COM port (Windows) or device name (Linux/Mac)
  - **Linux**: Usually `/dev/ttyUSB0` or `/dev/ttyACM0`
  - **Windows**: Usually `COM3`, `COM4`, etc.
  - **Mac**: Usually `/dev/tty.usbserial-*`

### Step 3: Start the Application

```bash
# Activate virtual environment first
source venv/bin/activate  # Linux/Mac
# or
venv\Scripts\activate.bat  # Windows

# Run the application
python main.py
```

### Step 4: Connect to Motor Controller

1. In the UI, select your serial port from the dropdown
2. Click "Connect"
3. Status should turn green: "Connected"

### Step 5: Control the Motor

1. **Start Motor**: Click "Start Motor" button (sends command `S`)
2. **Set Frequency**: Enter frequency in Hz, click "Set" (sends `F 50.0`)
3. **Set Amplitude**: Set voltage level 0.0-1.0, click "Set" (sends `A 0.5`)
4. **Monitor**: Watch the real-time plots and status values

## Testing Without Hardware

If you don't have the motor controller connected yet, you can test the UI:

### Using the Mock Controller

1. **Terminal 1** - Start the mock controller:
```bash
python mock_controller.py /dev/ttyS0
```

2. **Terminal 2** - Start the UI:
```bash
source venv/bin/activate
python main.py
```

3. In the UI, connect to `/dev/ttyS0` (Linux) or `COM1` (Windows)

> **Note**: On Linux, you may need to create a virtual serial port pair:
> ```bash
> sudo socat -d -d pty,raw,echo=0 pty,raw,echo=0
> ```
> This creates a virtual port pair for testing

## Common Issues

### "No serial ports found"
- Check USB cable connection
- Restart the application
- Try a different USB port
- On Linux: `lsusb` shows connected devices

### "Failed to connect"
- Ensure another program isn't already using the port
- Try restarting the motor controller
- Check that baudrate is 115200

### "No data received"
- Motor controller must be sending telemetry data
- Check format: `SPD:1234.56 CUR:0.12,-0.11`
- Verify connection is active (doesn't just say "Connected")

## Key Commands

| Button | Command | Purpose |
|--------|---------|---------|
| Set Frequency | `F 50.0` | Set inverter frequency |
| Set Amplitude | `A 0.5` | Set output voltage level |
| Set RPM | `R 1500` | Set target speed |
| Start Motor | `S` | Begin motor rotation |
| Stop Motor | `X` | Stop motor safely |

## Real-Time Monitoring

The UI automatically displays:
- **Speed**: Actual motor speed in RPM
- **Phase Currents**: Ia and Ib in Amperes
- **Applied Torque**: Calculated from current magnitude
- **EM Torque**: Estimated from speed dynamics

## Plots

Three tabs show live data:
1. **Speed (RPM)** - Actual vs Reference
2. **Torque (N⋅m)** - Applied vs Electromagnetic
3. **Phase Currents (A)** - Ia and Ib waveforms

Data updates automatically as telemetry arrives.

## File Structure

```
Motor-Control-UI/
├── main.py                 ← Run this file
├── app_window.py          (UI definition)
├── serial_communicator.py  (Serial communication)
├── data_buffer.py         (Data storage & calculations)
├── config.py              (Settings)
├── mock_controller.py     (Testing tool)
└── README.md              (Full documentation)
```

## Next Steps

1. **Calibrate Motor Constants**: Edit `MOTOR_MOMENT_OF_INERTIA` and friction values in `config.py`
2. **Customize Limits**: Adjust `FREQUENCY_MAX`, `AMPLITUDE_MAX`, `RPM_MAX` as needed
3. **Data Logging**: Modify the application to save data to CSV files
4. **Advanced Features**: Implement FFT analysis, tuning algorithms, etc.

## Support

- Check the message log in the UI for error details
- Review [README.md](README.md) for comprehensive documentation
- Verify the communication protocol section matches your motor controller

---

**Ready?** Start with `python main.py` and select your serial port! 🚀
