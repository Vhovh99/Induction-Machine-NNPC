# Induction Machine FOC — Binary Serial Protocol

## Physical Layer

| Parameter | Value |
|-----------|-------|
| Interface | LPUART1 |
| Baud rate | 230 400 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow ctrl | None |

## Frame Format

Every frame (both directions) uses the same envelope:

```
┌──────┬─────┬─────┬────────────────────┬──────┐
│ SYNC │ CMD │ LEN │ PAYLOAD (0..56 B)  │ CRC8 │
│ 1 B  │ 1 B │ 1 B │    LEN bytes       │ 1 B  │
└──────┴─────┴─────┴────────────────────┴──────┘
```

| Field | Size | Description |
|-------|------|-------------|
| SYNC | 1 | Always `0xAA` |
| CMD | 1 | Command or response ID (see tables below) |
| LEN | 1 | Payload length in bytes (0–56) |
| PAYLOAD | 0–56 | Command-specific data |
| CRC8 | 1 | CRC-8/MAXIM computed over **CMD + LEN + PAYLOAD** |

### CRC-8/MAXIM

- **Polynomial:** `0x31` (x⁸ + x⁵ + x⁴ + 1)
- **Init value:** `0x00`
- **Input/Output reflection:** None
- **Scope:** Computed over bytes `CMD`, `LEN`, and all `PAYLOAD` bytes (excludes SYNC)

Reference implementation (C):

```c
uint8_t crc8_maxim(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}
```

Reference implementation (Python):

```python
def crc8_maxim(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc
```

---

## Commands (Host → MCU)

All multi-byte numeric values are **little-endian**. Floats are IEEE 754 single-precision (4 bytes).

### 0x01 — MOTOR_START

Start the motor. Triggers the flux-build → running state machine.

| Field | Value |
|-------|-------|
| CMD | `0x01` |
| LEN | `0` |
| Payload | (none) |

**Response:** ACK (`0x80`)

---

### 0x02 — MOTOR_STOP

Stop the motor and zero speed/torque references.

| Field | Value |
|-------|-------|
| CMD | `0x02` |
| LEN | `0` |
| Payload | (none) |

**Response:** ACK (`0x80`)

---

### 0x03 — SET_SPEED_REF

Set the speed reference (outer loop).

| Field | Value |
|-------|-------|
| CMD | `0x03` |
| LEN | `4` |
| Payload | `float omega_ref` — mechanical speed in **rad/s** |

**Response:** ACK (`0x80`)

Example: Set 100 rad/s → payload = `0x00 0x00 0xC8 0x42` (IEEE 754 LE for 100.0)

---

### 0x04 — SET_ID_REF

Set the d-axis (flux) current reference.

| Field | Value |
|-------|-------|
| CMD | `0x04` |
| LEN | `4` |
| Payload | `float id_ref` — d-axis current in **A** |

**Response:** ACK (`0x80`)

---

### 0x05 — SET_CURRENT_PI

Set the current PI controller gains (applied to both d and q axes).

| Field | Value |
|-------|-------|
| CMD | `0x05` |
| LEN | `8` |
| Payload | `float Kp` (4B) + `float Ki` (4B) |

**Response:** ACK (`0x80`)

---

### 0x06 — SET_SPEED_PI

Set the speed PI controller gains.

| Field | Value |
|-------|-------|
| CMD | `0x06` |
| LEN | `8` |
| Payload | `float Kp` (4B) + `float Ki` (4B) |

**Response:** ACK (`0x80`)

---

### 0x07 — GET_STATUS

Request a one-shot status packet.

| Field | Value |
|-------|-------|
| CMD | `0x07` |
| LEN | `0` |
| Payload | (none) |

**Response:** STATUS (`0x83`) — see below.

---

### 0x08 — SET_TELEMETRY_DIV

Configure the telemetry stream rate. Telemetry is sent every `divider` iterations of the main loop. Set to `0` to disable telemetry streaming.

| Field | Value |
|-------|-------|
| CMD | `0x08` |
| LEN | `2` |
| Payload | `uint16_t divider` (LE) |

**Response:** ACK (`0x80`)

The main loop runs at approximately the UART processing rate. Default divider is **200**.

---

## Responses (MCU → Host)

### 0x80 — ACK

Acknowledges a successful command.

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `cmd_echo` — the CMD byte that was acknowledged |

---

### 0x81 — NACK

Indicates a command was rejected.

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | `cmd_echo` — the CMD byte that was rejected |
| 1 | 1 | `error_code` |

Error codes:

| Code | Name | Description |
|------|------|-------------|
| `0x01` | `ERR_UNKNOWN_CMD` | Unrecognized command ID |
| `0x02` | `ERR_BAD_LENGTH` | Payload length mismatch for command |
| `0x03` | `ERR_BAD_CRC` | CRC-8 check failed |
| `0x04` | `ERR_INVALID_VALUE` | Value out of permissible range |

---

### 0x82 — TELEMETRY

Periodic telemetry stream packet (28 bytes payload).

| Offset | Size | Type | Field | Unit |
|--------|------|------|-------|------|
| 0 | 4 | float | `id` | A (d-axis current, filtered) |
| 4 | 4 | float | `iq` | A (q-axis current, filtered) |
| 8 | 4 | float | `vbus` | V (DC bus voltage) |
| 12 | 4 | float | `omega_m` | rad/s (mechanical speed) |
| 16 | 4 | float | `ia` | A (phase A current) |
| 20 | 4 | float | `ib` | A (phase B current) |
| 24 | 4 | float | `ic` | A (phase C current) |

---

### 0x83 — STATUS

One-shot status response (12 bytes payload).

| Offset | Size | Type | Field | Description |
|--------|------|------|-------|-------------|
| 0 | 1 | uint8 | `motor_state` | 0=IDLE, 1=FLUX_BUILD, 2=RUNNING |
| 1 | 4 | float | `omega_ref` | Current speed reference (rad/s) |
| 5 | 4 | float | `id_ref` | Current flux reference (A) |
| 9 | 1 | uint8 | `fault_flags` | Bitfield (reserved, currently 0) |
| 10 | 2 | uint16 | `uptime_s` | MCU uptime in seconds |

---

## Example Transaction

**Host sends SET_SPEED_REF = 50.0 rad/s:**

```
TX: AA 03 04 00 00 48 42 <CRC>
     │  │  │  └──────────┘  └─ CRC-8 over [03 04 00 00 48 42]
     │  │  │   float 50.0 LE
     │  │  └─ LEN=4
     │  └─ CMD=0x03
     └─ SYNC
```

**MCU responds ACK:**

```
RX: AA 80 01 03 <CRC>
     │  │  │  │   └─ CRC-8 over [80 01 03]
     │  │  │  └─ cmd_echo=0x03
     │  │  └─ LEN=1
     │  └─ RSP_ACK
     └─ SYNC
```

---

## State Machine (RX)

The MCU receiver is a byte-level state machine driven by UART interrupt:

```
SYNC ──(0xAA)──► CMD ──(any)──► LEN ──(0..56)──► PAYLOAD ──(N bytes)──► CRC
  ▲                                                                        │
  └────────────────────────────────────────────────────────────────────────┘
                              (reset on invalid LEN or CRC mismatch)
```

If `LEN > 56` or CRC doesn't match, the frame is silently dropped and the state machine resets to SYNC.

---

## Integration Notes

- The protocol uses **LPUART1** (`hlpuart1` handle in firmware).
- Telemetry is sent from the `while(1)` main loop — not from the 20 kHz ISR — so effective telemetry rate depends on the divider and main loop throughput.
- `proto_speed_ref` and `proto_id_ref` are `volatile float` globals written by the protocol handler and read by the FOC ISR — no mutex needed on Cortex-M4 (atomic 32-bit aligned writes).
- Motor start triggers the state machine: IDLE → FLUX_BUILD (0.2 s) → RUNNING.
- Motor stop immediately sets state to IDLE and zeros references.
- Default `id_ref` on boot is **0.4 A**, default `speed_ref` is **0.0 rad/s**, default telemetry divider is **200**.
