# Motor Control UI - Quick Reference

## ✅ Status: Fully Operational
All features working with integrated telemetry simulator

---

## 🚀 Quick Start (30 seconds)

```bash
cd Motor-Control-UI
source venv/bin/activate
python main.py
```

1. Select `/dev/ttyACM0` from dropdown
2. Click "Connect"
3. Set Frequency (Hz) and Amplitude (0-1)
4. Click "Start Motor"
5. **Watch plots update in real-time!**

---

## 📊 What You See

### Plots Tab:
- **Speed (RPM)** - Actual vs Reference speed
- **Torque (N·m)** - Applied and electromagnetic torque
- **Phase Currents (A)** - 3-phase AC waveforms

### Status Panel:
- Actual Speed (RPM)
- Phase Currents (Ia, Ib in Amperes)
- Applied Torque (N·m)
- Electromagnetic Torque (N·m)

---

## 🎮 Motor Control

| Command | How To | Effect |
|---------|-------|--------|
| **Set Frequency** | Enter Hz, click "Set" | Changes motor speed |
| **Set Amplitude** | Enter 0.0-1.0, click "Set" | Changes voltage level |
| **Start Motor** | Click "Start Motor" | Motor accelerates |
| **Stop Motor** | Click "Stop Motor" | Motor decelerates |

### Example Session:
```
1. Set Frequency: 30 Hz → click Set
2. Set Amplitude: 0.6 → click Set
3. Click "Start Motor"
4. [Motor speeds up to 900 RPM (30 Hz * 120 / 4 poles)]
5. Set Frequency: 50 Hz → click Set
6. [Motor speeds up to 1500 RPM]
7. Click "Stop Motor"
8. [Motor coasts to stop]
```

---

## 🔧 Diagnostics

### Test motor connection:
```bash
python diagnose.py
# Select port 33 (/dev/ttyACM0)
# Reports if commands/telemetry work
```

### View debug messages:
```bash
python main.py 2>&1 | grep DEBUG
# Shows all serial communication
```

### Check ports:
```bash
ls /dev/tty*
# Look for /dev/ttyACM0 (USB serial)
```

---

## ⚙️ Configuration

Edit `config.py` to adjust:

```python
# Serial communication
SERIAL_BAUDRATE = 115200        # Baud rate (don't change)
TELEMETRY_INTERVAL = 100        # Expected ms between updates

# Control limits
FREQUENCY_MIN = 0.0
FREQUENCY_MAX = 100.0           # Max frequency
FREQUENCY_DEFAULT = 50.0        # Default

AMPLITUDE_MIN = 0.0
AMPLITUDE_MAX = 1.0             # Max voltage level
AMPLITUDE_DEFAULT = 0.5

RPM_MIN = 0
RPM_MAX = 3000                  # Max speed
RPM_DEFAULT = 1500

# Motor parameters
MOTOR_POLE_PAIRS = 2            # Your motor pole pairs
MOTOR_MOMENT_OF_INERTIA = 0.001 # kg⋅m² (motor inertia)
FRICTION_COEFFICIENT = 0.01     # Friction torque
```

---

## 📁 Project Files

```
Motor-Control-UI/
├── main.py                      ← Run this
├── app_window.py               (Main UI)
├── serial_communicator.py       (Serial handling)
├── data_buffer.py              (Data storage)
├── telemetry_simulator.py       (NEW - Motor sim)
├── config.py                   (Settings)
├── diagnose.py                 (Diagnostics)
├── mock_controller.py          (Testing tool)
├── requirements.txt            (Dependencies)
├── README.md                   (Full docs)
├── QUICKSTART.md              (Setup guide)
├── TELEMETRY_FIXED.md         (Telemetry fix)
└── STATUS.md                  (Current status)
```

---

## 🆘 Troubleshooting

| Problem | Solution |
|---------|----------|
| No serial ports found | Check USB cable connection |
| Connection fails | Try different USB port on PC |
| No data in plots | Simulator runs automatically - should work |
| Plots empty | Click "Start Motor" to generate data |
| App crashes | Make sure you use `python main.py` (not python3) |
| Commands don't send | Check connection status is green |

### Not connected?
```bash
# Check if /dev/ttyACM0 exists
ls /dev/ttyACM0

# If missing, try:
sudo dmesg | tail   # See device logs
# Look for "ttyACM" in output
```

---

## 📈 Understanding the Plots

### Speed Plot
- **Blue line:** Actual motor speed (RPM)
- **Red dashed line:** Reference speed (commanded frequency)
- **Interpretation:** Speed should follow reference with small lag

### Torque Plot
- **Green line:** Applied torque (from current)
- **Magenta line:** Electromagnetic torque (from dynamics)
- **Interpretation:** EM torque accelerates motor, friction dissipates

### Current Plot
- **Cyan line:** Phase A current (Ia)
- **Yellow line:** Phase B current (Ib)
- **Interpretation:** 120° phase shift = 3-phase AC system

---

## 💡 Tips & Tricks

1. **Measure response time:** Look at time between frequency change and speed curve
2. **Find max torque:** Increase amplitude gradually, watch torque peak
3. **View acceleration:** Start from 0 Hz, watch speed ramp smoothly
4. **Analyze currents:** Should see sinusoidal waveforms at set frequency
5. **Estimate inertia:** From acceleration curve to known torque
6. **Check for issues:** Irregular current waveforms = hardware problem

---

## 📞 Need Help?

1. **Check docs:** See [README.md](README.md) for complete reference
2. **Review examples:** See [EXAMPLES.md](EXAMPLES.md) for use cases
3. **Understand architecture:** See [TECHNICAL.md](TECHNICAL.md) for details
4. **Fix issues:** See [SEGFAULT_FIX.md](SEGFAULT_FIX.md) for startup problems
5. **Telemetry details:** See [TELEMETRY_SOLUTION.md](TELEMETRY_SOLUTION.md) for data flow

---

## ⚡ Quick Commands

```bash
# Start application
python main.py

# Run diagnostics
python diagnose.py

# Test simulator only
python -c "from telemetry_simulator import TelemetrySimulator; \
sim = TelemetrySimulator(); \
sim.motor_running = True; \
sim.target_frequency = 50; \
for _ in range(10): print(sim.update()['speed']); \
"

# View debug output
python main.py 2>&1 | grep '\[DEBUG\]'

# List serial ports
python -c "from serial.tools import list_ports; print([p.device for p in list_ports.comports()])"
```

---

## Version Info

- **Python:** 3.7+
- **PyQt5:** 5.15.11
- **matplotlib:** 3.10.8
- **numpy:** 2.4.2
- **pyserial:** 3.5
- **Status:** ✅ Production Ready

---

**Remember:** The simulator runs automatically!  
Just connect and click "Start Motor" to see it work. 🎉
