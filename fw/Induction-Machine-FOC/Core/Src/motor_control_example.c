/**
 * @file motor_control_example.c
 * @brief Complete example implementation for 3-phase AC induction motor control
 *
 * This file demonstrates various ways to use the motor control API
 * Copy functions from here into your main.c as needed
 */

#include "main.h"

/* ============================================================================
 * SIMPLE START EXAMPLE - Start motor with fixed frequency and amplitude
 * ============================================================================
 */
void Example_Simple_Start(void) {
  // Initialize and start motor
  Motor_Start();

  // Set frequency to 15 Hz
  Motor_SetFrequency(15.0f);

  // Set amplitude to 60% for reasonable torque
  Motor_SetAmplitude(0.6f);
}

/* ============================================================================
 * SOFT START EXAMPLE - Gradual acceleration from 0 to full speed
 * ============================================================================
 */
void Example_Soft_Start(void) {
  Motor_Start();
  Motor_SetFrequency(2.0f); // Start at low frequency

  // Gradually increase amplitude to avoid inrush current
  for (float amplitude = 0.0f; amplitude <= 1.0f; amplitude += 0.01f) {
    Motor_SetAmplitude(amplitude);

    // Wait 100ms per step (total ramp: ~10 seconds)
    for (int i = 0; i < 100; i++) {
      Motor_Update();
      HAL_Delay(1);
    }
  }

  // Now motor is running at full amplitude
  Motor_SetFrequency(30.0f); // Can increase frequency now
}

/* ============================================================================
 * V/f CONTROL EXAMPLE - Maintain constant voltage-to-frequency ratio
 * ============================================================================
 * This is the most common method for simple induction motor control
 * Voltage = k * Frequency (constant torque, variable speed)
 */
void Example_VF_Control_Ramp(void) {
  Motor_Start();

  // V/f characteristic: 0.02 V per Hz (adjust based on your motor)
  float vf_constant = 0.02f;
  float target_frequency = 50.0f;

  // Ramp frequency from 0 to target
  for (float freq = 0.0f; freq <= target_frequency; freq += 0.5f) {
    Motor_SetFrequency(freq);

    // Maintain V/f ratio: voltage = frequency * constant
    float voltage = freq * vf_constant;
    if (voltage > 1.0f)
      voltage = 1.0f; // Clamp to maximum
    if (voltage < 0.1f)
      voltage = 0.1f; // Minimum voltage to overcome friction

    Motor_SetAmplitude(voltage);

    // Ramp over 20 seconds
    for (int i = 0; i < 500; i++) {
      Motor_Update();
      HAL_Delay(1);
    }
  }
}

/* ============================================================================
 * FREQUENCY SWEEP EXAMPLE - Test response at different frequencies
 * ============================================================================
 */
void Example_Frequency_Sweep(void) {
  Motor_Start();
  Motor_SetAmplitude(0.7f); // Keep voltage constant

  // Sweep from 5 Hz to 45 Hz in 1 Hz steps
  for (float freq = 5.0f; freq <= 45.0f; freq += 1.0f) {
    Motor_SetFrequency(freq);

    // Hold each frequency for 2 seconds
    for (int i = 0; i < 2000; i++) {
      Motor_Update();
      HAL_Delay(1);
    }
  }

  Motor_Stop();
}

/* ============================================================================
 * ACCELERATION PROFILE EXAMPLE - Smooth S-curve acceleration
 * ============================================================================
 */
void Example_SCurve_Acceleration(void) {
  Motor_Start();
  Motor_SetAmplitude(0.5f);

  // Phase 1: Soft start acceleration (0-20 Hz, 3 seconds)
  for (int t = 0; t < 3000; t += 10) {
    float freq = 20.0f * (t / 3000.0f); // Linear ramp
    Motor_SetFrequency(freq);

    for (int i = 0; i < 10; i++) {
      Motor_Update();
      HAL_Delay(1);
    }
  }

  // Phase 2: Constant acceleration (20-40 Hz, 2 seconds)
  for (int t = 0; t < 2000; t += 10) {
    float freq = 20.0f + 20.0f * (t / 2000.0f); // Linear ramp
    Motor_SetFrequency(freq);

    for (int i = 0; i < 10; i++) {
      Motor_Update();
      HAL_Delay(1);
    }
  }

  // Phase 3: Deceleration (40-30 Hz, 1 second)
  for (int t = 0; t < 1000; t += 10) {
    float freq = 40.0f - 10.0f * (t / 1000.0f); // Linear ramp down
    Motor_SetFrequency(freq);

    for (int i = 0; i < 10; i++) {
      Motor_Update();
      HAL_Delay(1);
    }
  }

  // Hold at 30 Hz for 5 seconds
  Motor_SetFrequency(30.0f);
  for (int i = 0; i < 5000; i++) {
    Motor_Update();
    HAL_Delay(1);
  }

  Motor_Stop();
}

/* ============================================================================
 * INTERACTIVE CONTROL EXAMPLE - Remote frequency/amplitude adjustment
 * ============================================================================
 * Use with a button or serial interface to control motor
 */
static float current_frequency = 0.0f;
static float current_amplitude = 0.0f;
static uint8_t motor_enabled = 0;

void Example_Interactive_Control_Init(void) {
  current_frequency = 0.0f;
  current_amplitude = 0.0f;
  motor_enabled = 0;
}

void Example_Interactive_Control_Update(void) {
  if (motor_enabled && current_frequency == 0.0f) {
    Motor_Start();
  } else if (!motor_enabled && current_frequency > 0.0f) {
    Motor_Stop();
  }

  Motor_SetFrequency(current_frequency);
  Motor_SetAmplitude(current_amplitude);
  Motor_Update();
}

// Call this when user presses UP button
void Example_Increase_Frequency(float step) {
  current_frequency += step;
  if (current_frequency > 50.0f) {
    current_frequency = 50.0f;
  }
}

// Call this when user presses DOWN button
void Example_Decrease_Frequency(float step) {
  current_frequency -= step;
  if (current_frequency < 0.0f) {
    current_frequency = 0.0f;
  }
  if (current_frequency < 1.0f) {
    motor_enabled = 0;
  }
}

// Call this when user presses START button
void Example_Motor_Enable(void) {
  motor_enabled = 1;
  current_frequency = 5.0f; // Start at 5 Hz
  current_amplitude = 0.3f;
}

// Call this when user presses STOP button
void Example_Motor_Disable(void) {
  motor_enabled = 0;
  current_frequency = 0.0f;
}

/* ============================================================================
 * MAIN LOOP INTEGRATION EXAMPLE
 * ============================================================================
 * This shows how to integrate the motor control into your main loop
 */
/*
void main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_HRTIM1_Init();

  HAL_Delay(100);

  // Choose one of the examples above:
  // Example_Simple_Start();
  // Example_Soft_Start();
  // Example_VF_Control_Ramp();
  // Example_Frequency_Sweep();
  // Example_SCurve_Acceleration();

  // Or for interactive:
  // Example_Interactive_Control_Init();

  while (1)
  {
    // Main control loop at 1 kHz
    Motor_Update();

    // Add other tasks here (ADC reading, serial communication, etc.)
    // But keep total time per iteration close to 1ms

    HAL_Delay(1);
  }
}
*/

/* ============================================================================
 * NOTES:
 *
 * 1. Always call Motor_Update() at a fixed rate (1 kHz recommended)
 * 2. Frequency range: 0.1 Hz to 50 Hz (motor dependent)
 * 3. Amplitude range: 0.0 to 1.0 (0% to 100% voltage)
 * 4. Use soft-start to reduce inrush current on DC link
 * 5. Monitor DC bus voltage - it should remain stable during operation
 * 6. The IPMM15B board handles power stage switching and protection
 * 7. Implement current limiting in your control logic if needed
 * 8. For better performance, tune V/f constant based on motor nameplate
 *
 * ============================================================================
 */
