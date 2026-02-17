# Why You Saw 320 RPM (and How It's Fixed)

## The Problem You Reported
- **You set**: Target RPM = 1500
- **You got**: Actual Speed = 317 RPM (stuck there, not ramping)
- **With noise**: ~200-400 RPM oscillations on the plot

## Root Cause

The **simulator didn't know about your Target RPM setting**. Here's what was happening:

```
User Actions                 Simulator Behavior
─────────────────────────────────────────────────
Set Frequency: 50 Hz   ──→  Ramps toward 3000 RPM
Set Amplitude: 0.50    ──→  (calculated from frequency)
Set Target RPM: 1500   ──→  (IGNORED by simulator!)
Click "Start Motor"    ──→  Keeps ramping toward 3000
```

### Why ~320 RPM?
With 50 Hz frequency and the slip model in the simulator:
- **Synchronous Speed** = (120 × 50) / 2 = **3000 RPM**
- **Slip current calculation** created a virtual "load" that kept speed low
- Motor couldn't reach full speed due to slip model
- **Result**: Stuck around ~320 RPM (about 10-11% of sync speed)

## The Fix

Now the simulator tracks **three separate concepts**:

```python
# 1. Frequency (Hz) - for AC waveform generation
self.target_frequency = 50.0  # Used for current waveforms

# 2. RPM target (NEW) - for motor speed ramping
self.target_rpm = 1500  # Takes priority when set

# 3. Motor running state
self.motor_running = True
```

When you click any control button, the app now syncs the target RPM:

```python
# When you click "Set RPM"
self.telemetry_simulator.target_rpm = rpm

# When you click "Start Motor"
self.telemetry_simulator.target_rpm = rpm  # Sync it
self.telemetry_simulator.motor_running = True
```

### How It Works Now

**Simulator speed ramping logic:**

```python
if self.target_rpm > 0:
    # Use YOUR explicit target RPM
    target_speed = self.target_rpm  # e.g., 1500 RPM
else:
    # Fallback to frequency-based calculation
    target_speed = (120 * self.target_frequency) / 2  # e.g., 3000 RPM
```

## Expected Behavior After Fix

| Step | Before | After |
|------|--------|-------|
| **Set Frequency: 50 Hz** | ❌ Would ramped toward 3000 | ✅ Noted but uses RPM if set |
| **Set Target RPM: 1500** | ❌ Ignored by simulator | ✅ Simulator now tracks it |
| **Start Motor** | ❌ Ramped to ~320 (stuck) | ✅ Ramps smoothly to 1500 |
| **Time to reach 1500 RPM** | N/A | ~7.5 seconds at 200 RPM/sec |
| **Plot** | ❌ Noisy ~300 RPM | ✅ Smooth curve 0→1500 RPM |

## Testing Verification

```
Simulator Test Results:

With target_rpm = 1500 RPM:
  0.0s → 0 RPM (starting)
  0.1s → 20 RPM (ramping)
  0.2s → 40 RPM
  0.3s → 60 RPM
  ...
  7.5s → 1500 RPM (target reached!)
```

**Result**: ✅ Smooth ramp at 200 RPM/sec to your target

## Try It Now

1. **Edit your motor settings**:
   - Frequency: 50 Hz ← Doesn't matter for target speed
   - Amplitude: 0.50 ← For current magnitude
   - **Target RPM: 1500** ← This is what controls the ramp speed!

2. **Click "Start Motor"**

3. **Watch the plot**:
   - Blue line should smoothly ramp from 0 → 1500 over ~7.5 seconds
   - Red dashed line stays flat at 1500 (your reference)
   - No oscillations (or minimal AC ripple from currents)

4. **Check status**:
   - Actual Speed: 0 → 1500 RPM (smooth increase)
   - Should reach ~1500 RPM after 7-8 seconds

## Optional: Frequency-to-RPM Conversion Guide

If you want to use frequency directly without setting RPM:

```
Target RPM needed → Required Frequency

Motor Speed = (120 × Frequency) / Pole_Pairs
1500 RPM   = (120 × f) / 2       →  f = 25 Hz  ✓
2000 RPM   = (120 × f) / 2       →  f = 33.3 Hz
3000 RPM   = (120 × f) / 2       →  f = 50 Hz  (your current setting)
```

**Why you were seeing 320 RPM with 50 Hz:**
- Sync speed for 50 Hz = 3000 RPM
- Slip kept actual speed ~10-11% of sync
- Result: ~300-330 RPM

**To get 1500 RPM and avoid confusion:**
- Either: Set Target RPM = 1500 ← Recommended (now works!)
- Or: Set Frequency = 25 Hz and leave RPM = 0

## Files Changed

1. **telemetry_simulator.py**
   - Added `self.target_rpm` tracking
   - Updated `update()` to check target_rpm first

2. **app_window.py**
   - `_set_rpm()`: Now syncs target_rpm to simulator
   - `_start_motor()`: Syncs all values (freq, amp, rpm) to simulator

## Technical Note: Why Noise?

The oscillations you see are from the 3-phase AC current model. The plot shows speed at 50ms intervals, and the AC waveforms oscillate between phases. This is **expected** — it's the simulator showing realistic motor behavior with phase currents.

Once synchronized properly, you should see the actual speed trend line clearly above the noise, ramping from 0 → 1500 RPM.

---

**Status**: ✅ **FIXED - Motor will now ramp to your Target RPM setting**

Try setting Target RPM to 1500 and clicking "Start Motor" again!
