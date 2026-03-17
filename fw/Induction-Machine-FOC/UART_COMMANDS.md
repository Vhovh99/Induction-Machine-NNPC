# Induction Machine FOC - UART Command Reference

## Quick Command Reference

### Motor Control Commands

| Command | Format | Description | Example |
|---------|--------|-------------|---------|
| **Start** | `S` | Start motor (uses current mode: V/F or FOC) | `S` |
| **Stop** | `X` | Stop motor immediately | `X` |
| **Calibrate Sensors** | `C` | Calibrate current sensors (motor must be OFF) | `C` |

### V/F Control Commands

| Command | Format | Description | Example |
|---------|--------|-------------|---------|
| **Frequency** | `F <freq>` | Set output frequency in Hz | `F 50.0` |
| **Amplitude** | `A <ampl>` | Set modulation amplitude (0.0-1.0) | `A 0.8` |
| **RPM** | `R <rpm>` | Set speed (converted to frequency) | `R 1500` |

### FOC Control Commands

| Command | Format | Valid Range | Description | Example |
|---------|--------|-------------|-------------|---------|
| **Mode** | `M V` or `M F` | V/F or FOC | Switch control mode | `M F` |
| **Flux (Id)** | `I <current>` | 0.0-3.0A | Set direct axis current reference | `I 0.5` |
| **Torque (Iq)** | `Q <current>` | -5.0 to 5.0A | Set quadrature axis current reference | `Q 1.5` |

## Usage Examples

### Basic V/F Operation
```
S                    → Start motor (V/F mode)
F 50.0               → Set frequency to 50 Hz
A 0.7                → Set amplitude to 70%
C                    → Calibrate current sensors
X                    → Stop motor
```

### Switching to FOC
```
S                    → Start in V/F mode
M F                  → Switch to FOC mode
I 0.3                → Set flux current to 0.3A
Q 0.0                → Start with zero torque
Q 0.5                → Apply 0.5A torque demand
X                    → Stop
```

### Speed Control via RPM
```
S                    → Start
R 1000               → Set 1000 RPM (for 2-pole motor pairs)
A 0.8                → Set amplitude
X                    → Stop
```

## UART Settings

- **Baud Rate:** 115200 bps
- **Data Bits:** 8
- **Stop Bits:** 1
- **Parity:** None
- **Flow Control:** None

## Response Messages

### Successful Mode Switch
```
Mode: V/F            → Switched to Voltage/Frequency control
Mode: FOC            → Switched to Field Oriented Control
```

### Telemetry Output (sent every 100ms)
```
SPD:xyz.xx CUR:Ia,Ib → Speed (RPM), Phase currents (Amperes)
```

## FOC Parameter Notes

### Flux Current (Id)
- **Purpose:** Magnetizing induction machine
- **Typical Value:** 0.3A - 0.8A
- **Warning:** Too high → overheating, too low → weak torque
- **Set Before:** Starting load application

### Torque Current (Iq)
- **Purpose:** Producing motor torque
- **Typical Value:** 0.5A - 2.0A (depends on motor rating)
- **Note:** Can be negative for motoring/generating modes
- **Limit:** Motor rated current

## Safety Commands

| Scenario | Action |
|----------|--------|
| **Emergency Stop** | Press Ctrl+C or send `X` |
| **Sensor Error** | Send `C` and `X` immediately |
| **Overcurrent** | Motor stops automatically, send `X` to reset |
| **Thermal Shutdown** | Motor stops if temperature > 100°C |

## Troubleshooting Quick Guide

| Issue | Command to Check | Expected Result |
|-------|------------------|-----------------|
| Motor won't start | `S` then `F 20` | Motor should start at low speed |
| No speed feedback | Encoder connected? | Check encoder wiring |
| Current unbalanced | `C` | Recalibrate sensors |
| Mode switch fails | `M F` | Should see "Mode: FOC" response |
| Telemetry missing | Serial connection OK? | Should see speeds every 100ms |

## Command Format Notes

- All commands are case-insensitive (f = F, a = A, etc.)
- Floating point values use decimal point (3.14, not 3,14)
- Commands end with newline/carriage return (\r\n)
- Response time is typically < 1ms
- Maximum command length: 64 characters

## Power-Up Sequence Example

```
[Power On]
↓
[Wait 100ms for boot]
↓
C                    → Calibrate current sensors
↓
S                    → Start motor
↓
F 20                 → Set low speed (20 Hz)
↓
[Monitor telemetry output for SPD and CUR values]
↓
[When stable, can increase speed or switch to FOC]
```

## Advanced: PID Tuning (future)

Current implementation supports:
```
P 10.0 5.0 0.1      → Tune Id PID (Kp, Ki, Kd)
T 10.0 5.0 0.1      → Tune Iq PID (Kp, Ki, Kd)
```
*(Requires code modification - not available in UART interface yet)*

---

**Last Updated:** February 2025
**Firmware Version:** With FOC Implementation
