# IRFOC Implementation Review
**Date:** February 22, 2026  
**Motor:** Induction Machine (2 poles, 2000 PPR encoder)  
**Control Platform:** STM32G474 @ 20 kHz

---

## 1. ADC SYNCHRONIZATION WITH PWM ANALYSIS

### Current Implementation (main.c:1734)
```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // Injection triggered by HRTIM on PWM valley
    // Measures: Ia, Ib, Ic from 3-shunt topology
    CurrentSense_ReadWithShuntSelection(svpwm_output.shunt1, shunt2)
}
```

### ✅ STRENGTHS
1. **Injected ADC triggered by HRTIM:** Excellent - samples at PWM valley for minimum switching noise
2. **3-shunt topology:** Correct - all three phases measured, excellent for current reconstruction
3. **Shunt selection logic:** Smart - uses SVPWM output to select best two shunts dynamically
4. **20 kHz sampling:** Adequate for induction motor control
5. **Synchronization flag check:** Good - validates JEOS before processing

### ⚠️ ISSUES & CONCERNS
1. **ISR execution time: 40μs = 80% of 50μs period**
   - Phase 1: Current sensing, Clarke/Park transforms
   - Phase 2: PID computations, encoder angle update
   - **ROOT CAUSE:** Unclear - likely redundant angle computations
   - **RECOMMENDATION:** Profile each section separately

2. **Encoder angle read TWICE in ISR:**
   ```c
   Line 1760: theta_rotor = Encoder_GetElectricalAngleRad(&encoder, MOTOR_POLE_PAIRS);
   Line 1763: float encoder_angle = Encoder_GetElectricalAngleRad(&encoder, MOTOR_POLE_PAIRS);
   Line 1779: theta_feedback = Encoder_GetElectricalAngleRad(&encoder, MOTOR_POLE_PAIRS);  // THIRD TIME!
   ```
   **FIX:** Cache encoder angle once:
   ```c
   float theta_encoder = Encoder_GetElectricalAngleRad(&encoder, MOTOR_POLE_PAIRS);
   FOC_UpdateOpenLoop(CURRENT_LOOP_TS_SEC, theta_encoder);
   // ... reuse theta_encoder for feedback
   ```

3. **CORDIC sin/cos reconfiguration per cycle:**
   ```c
   if (CORDIC->CSR != csr) {
       CORDIC->CSR = csr;  // Reconfigure EVERY TIME
   }
   ```
   **FIX:** Configure once at init, never change (CSR value is always identical)

---

## 2. FLUX VECTOR ALIGNMENT WITH ROTOR POSITION

### Current Flow (foc_control.c)

#### **Closed-Loop Mode:**
```
theta_encoder (rotor position from encoder)
    ↓
theta_park = theta_encoder + theta_offset
    ↓
Park Transform of Ia, Ib, Ic → id, iq (current in d-q frame)
    ↓
Rotor Flux Model Updates: ψr = f(id, ψr_prev)
    ↓ 
Slip Calculation: ω_slip = (Lm/ψr) × (Rr/Lr) × iq
    ↓
theta_slip = theta_slip + ω_slip × dt
    ↓
theta_sync = theta_park + theta_slip (stator voltage angle)
    ↓
Inverse Park Transform → Vα, Vβ → SVPWM
```

### ✅ CORRECT THEORETICAL IMPLEMENTATION
1. **d-axis aligned with rotor flux:** ✓
   - Park transform uses `theta_park = theta_encoder + theta_offset`
   - Rotor flux observed in d-axis: `dψr/dt = (Rr/Lr) × (Lm×id - ψr)`

2. **Slip angle computed from torque current:** ✓
   ```c
   omega_slip = slip_gain × (Lm/ψr) × (Rr/Lr) × iq
   ```
   - Correctly decouples Id (flux) from Iq (torque)
   - **KEY FIX APPLIED:** Changed from `i_dq.q` to `Iq_ref` to avoid feedback oscillation

3. **Voltage generation ahead of rotor:** ✓
   - Inverse Park uses `theta_sync = theta_park + theta_slip`
   - Creates rotating field that leads rotor by slip angle

### ⚠️ CRITICAL ISSUES FOUND

#### **Issue #1: Slip Calculation Uses Measured Current (PARTIALLY FIXED)**
```c
Line 275: omega_slip = slip_gain × (Lm/ψr) × (Rr/Lr) × i_dq.q
```
**CURRENT STATE:** Changed to `Iq_ref` - ✓ GOOD FIX  
**WHY IMPORTANT:** Using measured `i_dq.q` adds delay and can cause oscillation:
- Iq measured → slip changes → voltage changes → Iq changes → measured again with 1-sample delay = feedback loop!

#### **Issue #2: Motor Parameters Likely Incorrect**
```c
foc_ctrl.Rr = 0.031f;   // Rotor resistance
foc_ctrl.Lr = 0.17f;    // Rotor inductance
foc_ctrl.Lm = 0.15f;    // Magnetizing inductance
```

**ANALYSIS:**
- Rr/Lr = 0.031/0.17 = 0.182 [1/s] → τr = 5.5ms (rotor time constant)
- Lm/Lr = 0.15/0.17 = 0.88 (very high, unusual)
- For typical 3-phase IM: Lm >> Lr, Ls (0.5-1.5H range)
- **Your motor:** Sub-watt range with very low inductances

**SYMPTOM:** If Rr or Lr are off by 50%, slip calculation is wrong:
- Slip too high → stator field rotates too fast → motor lags → vibrates
- Slip too low → stator field rotates too slow → motor overshoots → vibrates

#### **Issue #3: Calibration Theta_offset Sign Ambiguity**
```c
Line 471: foc_ctrl.theta_offset = foc_ctrl.calib_best_encoder - foc_ctrl.calib_best_offset;
```
**CURRENT STATE:** Inverted sign (you reverted after oscillation)  
**CORRECT FORMULA:** Should be:
```c
theta_offset = voltage_angle_best - encoder_reading_at_best
// or equivalently:
theta_offset = (angle_that_minimizes_Iq) - (encoder_position_at_that_moment)
```

#### **Issue #4: Slip Gain Default Too High for Small Motor**
```c
Line 151: foc_ctrl.slip_gain = 0.5f;  // Now reduced from 1.0
```

**ANALYSIS:** Slip gain multiplies the inherent slip formula:
- For rated load, motor slip ≈ Rr/ωr
- slip_gain > 1.0 OVERESTIMATES slip → vibration
- slip_gain < 1.0 UNDERESTIMATES slip → sluggish response

**RECOMMENDATION:** Start with `slip_gain = 0.3` for small motors to avoid overshoot

#### **Issue #5: PID Gains Still May Be Too Conservative**
```c
Line 121-130: Kp=0.2, Ki=5.0 (for both Id and Iq)
```
**CONCERN:** With 40μs ISR execution time + 50μs sample period = timing uncertainty:
- If ISR takes 45μs occasionally, next sample arrives only 5μs later!
- PID filters with Ki=5.0 may accumulate rapidly and cause overshoot
- Low Kp=0.2 with high Ki=5.0 causes lag then overshoot

**RECOMMENDED TUNING SEQUENCE:**
1. Increase Kp to 0.5 (faster response to errors)
2. Reduce Ki to 2.0 (slower integral accumulation)
3. Add Kd = 0.05 (damping)

---

## 3. DETAILED IRFOC FLOW ANALYSIS

### Correct Implementation Check:

| Component | Your Code | Theory | Status |
|-----------|-----------|--------|--------|
| **Clarke Transform** | `Clarke_Transform(Ia,Ib,Ic)` | iα = Ia; iβ = (Ia + 2Ib)/√3 | ✅ Standard |
| **Park Transform** | `theta_park = encoder + offset` | d-axis = rotor flux axis | ✅ Correct |
| **Current Feedback** | `Park_Transform(iα, iβ, sin(θ_park), cos(θ_park))` | Rotate iαβ to dq | ✅ Correct |
| **Rotor Flux Model** | `dψr/dt = (Rr/Lr)(Lm×id - ψr)` | Current model | ✅ Standard |
| **Slip Angle** | `ω_slip = (Lm/ψr)(Rr/Lr)Iq_ref` | Slip compensation | ✅ Correct |
| **Inverse Park** | `theta_sync = park + slip` | Voltage ahead of rotor | ✅ Correct |
| **Voltage Output** | `InvPark_Transform(Vd, Vq, sin(θ_sync))` | Generate 3-phase voltage | ✅ Correct |

---

## 4. ROOT CAUSE ANALYSIS: WHY VIBRATION?

### Theory vs Reality Mismatch:

**Most Likely Causes (in priority order):**

1. **Motor parameter values are wrong** (HIGH PROBABILITY)
   - Rr, Lr, Lm not matching your actual motor
   - Easy fix: Measure motor parameters or use pre-identification at startup

2. **Timing jitter from 40μs ISR** (MEDIUM PROBABILITY)
   - 80% CPU utilization leaves only 10μs margin
   - Occasional sample period variance = angle calculation error
   - Fix: Reduce ISR time to <30μs

3. **Theta_offset sign incorrectly inverted** (LOW PROBABILITY - you tested both)
   - If motor oscillates both ways, likely not the sign

4. **Slip gain or PID gains need readjustment** (MEDIUM PROBABILITY)
   - Coupled with wrong motor parameters

---

## 5. RECOMMENDED FIXES (PRIORITY ORDER)

### FIX #1: Eliminate Redundant Encoder Reads (Quick Win - saves ~5μs)
```c
// In HAL_ADCEx_InjectedConvCpltCallback (lines 1758-1779):

// Cache encoder angle ONCE
float theta_encoder = Encoder_GetElectricalAngleRad(&encoder, MOTOR_POLE_PAIRS);

// Reuse everywhere:
FOC_UpdateOpenLoop(CURRENT_LOOP_TS_SEC, theta_encoder);
// ...
theta_feedback = theta_encoder;  // reuse, don't call again
```

### FIX #2: Optimize CORDIC Configuration (saves ~3-5μs)
```c
// Move to FOC_Init():
csr = CORDIC_FUNCTION_SINE | CORDIC_PRECISION_6CYCLES | 
      CORDIC_SCALE_0 | CORDIC_NBWRITE_1 | CORDIC_NBREAD_2 | 
      CORDIC_INSIZE_32BITS | CORDIC_OUTSIZE_32BITS;
CORDIC->CSR = csr;  // Set once, never change

// In FOC_Update(), remove the "if" check:
// Just repeatedly call CORDIC_SinCos() - CSR stays constant
```

### FIX #3: Measure Motor Parameters
```bash
# Run these tests to identify Rr and Lr:

# Test 1: Stator resistance (simple)
- Disconnect motor, apply DC voltage
- Measure voltage and current between two phases
- Rs = V / I

# Test 2: Rotor time constant (requires oscilloscope or frequency sweep)
- Apply 10V AC at motor shaft = open-circuit, measure impedance vs frequency
- Or: Locked rotor test at 1 Hz - slowly increase V, record V/I at stall

# Quick approximation for small IM:
- If slot opening visible in rotor: estimate aluminum bar loss
- Rr ≈ Rs × (1-1.5)  [usually Rr < Rs]  
- Lr ≈ 0.8 × Ls
```

### FIX #4: Conservative Tuning Parameters
```c
// In FOC_Init():
foc_ctrl.slip_gain = 0.3f;      // Reduced (was 0.5)
foc_ctrl.pid_id.Kp = 0.3f;      // Slightly increased (was 0.2)
foc_ctrl.pid_id.Ki = 3.0f;      // Reduced (was 5.0)
foc_ctrl.pid_iq.Kp = 0.3f;
foc_ctrl.pid_iq.Ki = 3.0f;
```

### FIX #5: Add Startup Motor Parameter ID (Optional - Advanced)
```c
// Periodically estimate Rr/Lr from actual slip:
// If ω_rotor measured, and Iq_ref known:
// Rr/Lr ≈ omega_slip × ψr / (Lm × Iq_ref)
```

---

## 6. VIBRATION SOURCE VERIFICATION

### Test Sequence:

**Step 1: Check if it's PID or motor parameters**
```
Command: S 1 0.1       // Reduce slip_gain
Result:  If smoother → motor param issue (Rr/Lr wrong)
         If same → PID issue
```

**Step 2: Check if it's timing jitter**
```c
// Add in ISR:
uint32_t start_us = DWT->CYCCNT / 168;  // CPU cycles → microseconds
// ... FOC code ...
uint32_t elapsed_us = (DWT->CYCCNT - start_us) / 168;
if (elapsed_us > 40) { /* log */ }

// If elapsed_us varies ±5μs → timing jitter confirmed
```

**Step 3: Verify encoder calibration is correct**
```
Command: W              // Check theta_rotor changes smoothly
Expected: θ_rotor increases monotonically at ~1°/sample (for slow motion)
Result:  If jumps/stutters → encoder issue (not IRFOC)
```

---

## SUMMARY

Your IRFOC implementation is **theoretically CORRECT** but **practically UNSTABLE** due to:

1. **High ISR CPU usage (80%)** → timing uncertainty
2. **Likely wrong motor parameters** → slip calculation incorrect
3. **Conservative PID gains with high Ki** → slow response + overshoot tendency

**Immediate Actions:**
1. Cache encoder angle (save 3-5μs)
2. Reduce slip_gain to 0.3
3. Test with S command to identify if motor params or PID tuning
4. Measure actual motor Rr/Lr if possible

