# Motor Control UI - Usage Examples

This document provides practical examples of how to use the Motor Control UI application.

## Basic Workflow

### Example 1: Simple Motor Speed Control

**Scenario**: You want to run the motor at 50 Hz with 0.7 amplitude (70% voltage)

**Steps**:

1. **Start Application**
```bash
python main.py
```

2. **Connect to Motor**
   - Select serial port from dropdown (e.g., "COM3")
   - Click "Connect"
   - Wait for status to show "Connected" (green)

3. **Configure Settings**
   - Set Frequency: 50.0 Hz
   - Click "Set" next to Frequency
   - Set Amplitude: 0.7
   - Click "Set" next to Amplitude

4. **Start Motor**
   - Click "Start Motor"
   - Motor should accelerate to ~1500 RPM (synchronous speed)
   - Check plots to verify speed tracking

5. **Monitor**
   - Watch Speed plot: should show actual speed approaching reference
   - Watch Torque plot: applied torque should start high, decrease as speed stabilizes
   - Watch Current plot: phase currents should stabilize

### Example 2: Variable Speed Operation

**Scenario**: You want to vary the motor speed dynamically

**Steps**:

1. Start motor at 30 Hz:
   ```
   Frequency: 30.0 Hz
   Amplitude: 0.5
   Click "Start Motor"
   ```

2. Increase speed gradually:
   ```
   After 5 seconds:
     Frequency: 40.0 Hz
     Click "Set"
   
   After 5 more seconds:
     Frequency: 50.0 Hz
     Click "Set"
   
   After 5 more seconds:
     Frequency: 60.0 Hz
     Click "Set"
   ```

3. **Observe in Plots**:
   - Speed plot shows step-like increases
   - Torque plot shows transient responses
   - Current plot shows oscillations during transitions

## Advanced Usage

### Motor Characterization

**Goal**: Determine your motor's real-world constants

**Procedure**:

1. Run at constant frequency (e.g., 50 Hz, 0.5 amplitude):
   ```
   Frequency: 50.0
   Amplitude: 0.5
   Start Motor
   ```

2. Record data for 30 seconds:
   - Watch the "Current Status" panel
   - Note the steady-state speed (e.g., 1493 RPM)
   - Note average current (e.g., 0.85 A)

3. Perform step test:
   ```
   While running at 50 Hz:
     Set Amplitude to 0.8 (sudden increase)
     Watch the transient response
     Measure time to reach steady state
   ```

4. Calculate moment of inertia:
   - From speed step response, estimate rise time
   - J = torque_difference / angular_acceleration
   - Update `MOTOR_MOMENT_OF_INERTIA` in config.py

### Tuning Control Loop

**Scenario**: Motor oscillates around target speed

**Diagnosis**:
1. Start motor at 50 Hz
2. Watch Speed plot
3. If oscillating: gains may be too high on motor controller

**Solution**:
- Check message log for any errors
- Verify frequency is exactly what was set
- Try lower frequency to reduce oscillation
- Contact motor controller firmware developer

### Torque Analysis

**Goal**: Validate electromagnetic torque estimation

**Procedure**:

1. Run at constant speed (e.g., 50 Hz, full amplitude):
   ```
   Frequency: 50.0
   Amplitude: 1.0
   Start Motor
   ```

2. After 10 seconds, gradually increase load:
   ```
   Watch Torque plot:
   - Applied Torque = K * current magnitude
   - EM Torque = J * dω/dt + friction
   ```

3. With no load change:
   - Applied torque should be proportional to current
   - EM torque should be near friction torque (small)

4. With increasing load:
   - Speed starts to drop if load > motor output
   - EM torque increases to resist load
   - Applied torque increases (higher current)

## Troubleshooting Examples

### Issue: "No data received"

**Checklist**:
```python
# 1. Start motor (sends S command)
Click "Start Motor"

# 2. Check message log for errors
# Should see: "Motor started"

# 3. Wait 500ms for telemetry
# Look for speed and current values updating

# 4. If still nothing:
# Check serial connection is really established
# Verify motor controller is sending data
```

### Issue: Speed doesn't match target

**Example**: Set frequency to 50 Hz, but speed is only 1400 RPM (should be ~1500)

**Possible causes**:
1. **Load on motor**: If under mechanical load, speed will drop
   - Solution: Reduce load

2. **Pole pairs mismatch**: Synchronous speed = 120*f/P
   - If using P=3 instead of P=2:
   - Edit `MOTOR_POLE_PAIRS` in config.py
   - Recalculate reference speed

3. **Slip in motor**: Normal for induction motors under load
   - 50-100 RPM slip at full load is typical
   - This is expected behavior

### Issue: Current values look wrong

**Example**: Current shows as 0.05 A but motor is clearly running hard

**Possible causes**:
1. **Scaling issue**: K_motor constant may be wrong
   - Edit `K_motor` in data_buffer.py
   - Unit: N⋅m/A

2. **Sensor not working**: Check with motor controller directly
   - Serial terminal: see raw "CUR:" values

3. **Zero offset**: If showing negative current at no-load:
   - Normal for phase currents (they're AC)
   - Check average over time is near zero

## Testing Procedures

### Functional Test Checklist

- [ ] Application starts without errors
- [ ] Serial ports listed in dropdown
- [ ] Can connect to serial port
- [ ] "Connected" status indicator turns green
- [ ] Start/Stop buttons are enabled when connected
- [ ] Can set Frequency, Amplitude, RPM
- [ ] Motor responds to Start command
- [ ] Plots show live data
- [ ] Can Stop motor
- [ ] Status values update at ~10 Hz
- [ ] Can disconnect and reconnect

### Using Mock Controller for Testing

```bash
# Terminal 1: Start mock controller
python mock_controller.py /dev/ttyS0

# Terminal 2: Start UI
python main.py

# In UI: Select /dev/ttyS0 and Connect
# Then: Set Frequency, Start Motor, watch plots
```

## Real Motor Setup

### Hardware Requirements
- Motor controller with serial interface
- USB-to-Serial adapter (if needed)
- Motor + load (optional)
- 24V power supply (typical)

### Connection Steps
1. Power off all equipment
2. Connect USB/Serial cable to PC
3. Connect serial to motor controller
4. Connect motor and load
5. Apply power
6. Start application
7. Select correct COM port
8. Connect

### Safety Checklist
- [ ] All cables properly connected
- [ ] No exposed wires
- [ ] Load properly secured
- [ ] Emergency stop available (if needed)
- [ ] Motor can freely rotate
- [ ] Start with low amplitude (0.3) first
- [ ] Monitor temperature during operation

## Calibration Procedure

### Motor Constant Calibration

**Objective**: Determine correct K_motor value

**Procedure**:
1. Apply known load or use dynamometer
2. Run motor at 50 Hz, 0.7 amplitude
3. Measure actual torque (from load)
4. Read displayed "Applied Torque" from UI
5. Calculate: K_motor = actual_torque / displayed_torque
6. Update K_motor in data_buffer.py

**Example**:
```
Actual measured torque: 5.2 N⋅m
Current from display: 2.1 A
Formula: T = K * I
K = T / I = 5.2 / 2.1 = 2.48 N⋅m/A
Update in data_buffer.py: K_motor = 2.48
```

### Moment of Inertia Estimation

**Method 1: From step response**
```python
# Run motor at constant torque (constant current)
# Measure time to reach 63% of final speed T_63
# J ≈ T / (ω_final / T_63)
```

**Method 2: From coast-down**
```python
# Run at steady speed, then cut power
# Measure deceleration
# J = T_friction / (dω/dt)
```

## Data Logging (Manual Method)

To save data, take screenshots of plots or use Python:

```python
# Add to app_window.py
def export_data(self):
    import csv
    from datetime import datetime
    
    data = self.data_buffer.get_plot_data()
    filename = f"export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time(s)', 'Speed(RPM)', 'Speed_Ref(RPM)', 
                        'Torque_Applied(Nm)', 'Torque_EM(Nm)', 'Ia(A)', 'Ib(A)'])
        
        for i in range(len(data['time'])):
            writer.writerow([
                data['time'][i],
                data['speed_actual'][i],
                data['speed_reference'][i],
                data['torque_applied'][i],
                data['torque_electromagnetic'][i],
                data['current_ia'][i],
                data['current_ib'][i]
            ])
```

## Performance Validation

### Checking Application Performance

**Check CPU Usage**:
```bash
# Linux
top  # Look for python process

# Windows
taskmgr.exe  # Monitor python.exe CPU %
```
- Should be <5% at rest
- <10% during active plotting

**Check Memory Usage**:
- Should be ~100 MB
- Grows slowly over time (memory leak would show rapid growth)

**Check Serial Communication**:
```bash
# On Linux, monitor serial data
cat /dev/ttyUSB0 | od -c  # Raw bytes
```

## Integration with Motor Controller Firmware

### Expected Firmware Behavior

Your motor controller should:

1. **On receiving "F 50.0"**: Set frequency to 50 Hz
2. **On receiving "A 0.5"**: Set amplitude to 0.5 (50% voltage)
3. **On receiving "S"**: Enable motor output
4. **On receiving "X"**: Disable motor output (safe stop)
5. **Every 100ms**: Send "SPD:xxxx.xx CUR:x.xx,x.xx"

### Testing Firmware Compatibility

```python
# Manual test: Send commands with serial terminal
# Use Linux: picocom, minicom, or screen
# Use Windows: PuTTY or Tera Term

screen /dev/ttyUSB0 115200
F 50.0  # Should set frequency
# Wait 100ms
# Should see: SPD:0.00 CUR:0.00,0.00  (or similar)
```

---

For more detailed information, see the main [README.md](README.md) or [TECHNICAL.md](TECHNICAL.md).
