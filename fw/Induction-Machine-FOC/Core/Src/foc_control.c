#include "foc_control.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <math.h>

static FOC_Control_t foc_ctrl = {0};
static const float FOC_TWO_PI = 6.28318530718f;
static const float FOC_PI = 3.14159265359f;
static const float CORDIC_Q31_PER_PI = 683565275.5768f;
static const float CORDIC_Q31_INV = 1.0f / 2147483648.0f;

static inline void CORDIC_SinCos(float angle, float *sin_out, float *cos_out)
{
    // Normalize to [-pi, pi] for Q1.31 input range.
    while (angle >= FOC_TWO_PI) {
        angle -= FOC_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += FOC_TWO_PI;
    }
    if (angle > FOC_PI) {
        angle -= FOC_TWO_PI;
    }

    int32_t angle_q31 = (int32_t)(angle * CORDIC_Q31_PER_PI);

    uint32_t csr = CORDIC_FUNCTION_SINE |
                   CORDIC_PRECISION_6CYCLES |
                   CORDIC_SCALE_0 |
                   CORDIC_NBWRITE_1 |
                   CORDIC_NBREAD_2 |
                   CORDIC_INSIZE_32BITS |
                   CORDIC_OUTSIZE_32BITS;

    if (CORDIC->CSR != csr) {
        CORDIC->CSR = csr;
    }

    CORDIC->WDATA = (uint32_t)angle_q31;
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) {
    }

    int32_t sin_q31 = (int32_t)CORDIC->RDATA;
    int32_t cos_q31 = (int32_t)CORDIC->RDATA;

    *sin_out = (float)sin_q31 * CORDIC_Q31_INV;
    *cos_out = (float)cos_q31 * CORDIC_Q31_INV;
}

static float WrapAngle0To2Pi(float angle)
{
    while (angle >= FOC_TWO_PI) {
        angle -= FOC_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += FOC_TWO_PI;
    }
    return angle;
}

/**
 * @brief Simple PID controller implementation
 */
static float PID_Update(PID_Controller_t *pid, float error, float ts)
{
    // Proportional term
    float p_term = pid->Kp * error;
    
    // Derivative term
    float d_term = 0.0f;
    if (pid->Kd != 0.0f) {
        d_term = pid->Kd * (error - pid->prev_error) / ts;
    }
    pid->prev_error = error;

    // Integral term with clamp anti-windup
    pid->integral += pid->Ki * error * ts;

    // Clamp integral such that output can still stay within limits
    float i_max = pid->output_max - p_term - d_term;
    float i_min = pid->output_min - p_term - d_term;
    if (pid->integral > i_max) pid->integral = i_max;
    if (pid->integral < i_min) pid->integral = i_min;

    float output = p_term + pid->integral + d_term;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

/**
 * @brief Reset PID controller state
 */
static void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void FOC_Init(void)
{
    memset(&foc_ctrl, 0, sizeof(FOC_Control_t));
    
    // Default current references
    foc_ctrl.Id_ref = 0.3f;  // Start with modest flux (0.3A)
    foc_ctrl.Iq_ref = 0.0f;  // Start with zero torque
    
    // DC Bus voltage defaults
    foc_ctrl.Vbus = 311.0f;      // Nominal bus voltage
    foc_ctrl.Vbus_min = 100.0f;  // Minimum operating voltage
    
    // Operating mode
    foc_ctrl.mode = FOC_MODE_OPEN_LOOP;
    foc_ctrl.openloop_state = OPENLOOP_IDLE;
    foc_ctrl.openloop_angle = 0.0f;
    foc_ctrl.openloop_freq = 0.0f;
    foc_ctrl.openloop_accel = 5.0f;  // 5 Hz/s default
    
    // PID parameters for Id controller (flux control)
    // Conservative tuning to avoid oscillations with 40μs ISR time
    foc_ctrl.pid_id.Kp = 0.3f;   // Slightly increased (was 0.2)
    foc_ctrl.pid_id.Ki = 3.0f;   // Reduced from 5.0 to avoid overshoot
    foc_ctrl.pid_id.Kd = 0.0f;
    foc_ctrl.pid_id.output_min = -10.0f;
    foc_ctrl.pid_id.output_max = 10.0f;
    
    // PID parameters for Iq controller (torque control)
    // Conservative tuning to avoid oscillations with 40μs ISR time
    foc_ctrl.pid_iq.Kp = 0.3f;   // Slightly increased (was 0.2)
    foc_ctrl.pid_iq.Ki = 3.0f;   // Reduced from 5.0 to avoid overshoot
    foc_ctrl.pid_iq.Kd = 0.0f;
    foc_ctrl.pid_iq.output_min = -10.0f;
    foc_ctrl.pid_iq.output_max = 10.0f;
    
    // Maximum output voltage (normalized, typically sqrt(2/3) for SVPWM = 0.8165)
    foc_ctrl.max_voltage = 0.8165f;

    // Induction machine parameters (IRFOC model defaults)
    foc_ctrl.Rs = 0.039f;
    foc_ctrl.Rr = 0.031f;
    // foc_ctrl.Lm = 1.36f;
    // foc_ctrl.Ls = 1.5540f;
    // foc_ctrl.Lr = 1.5540f;
    foc_ctrl.Lm = 0.15f;
    foc_ctrl.Ls = 0.17f;
    foc_ctrl.Lr = 0.17f;


    // IRFOC defaults (encoder-based rotor angle + slip compensation)
    foc_ctrl.slip_comp_enabled = 1U;  // Enable slip compensation (essential for IRFOC)
    foc_ctrl.slip_gain = 0.3f;  // Reduced from 0.5 - smaller motor needs less slip boost
    foc_ctrl.min_id_for_slip = 0.2f;
    foc_ctrl.min_flux_wb = foc_ctrl.Lm * foc_ctrl.min_id_for_slip;
    foc_ctrl.rotor_flux_wb = foc_ctrl.Lm * foc_ctrl.Id_ref;
    foc_ctrl.omega_slip = 0.0f;
    foc_ctrl.theta_slip = 0.0f;
    foc_ctrl.theta_sync = 0.0f;
    foc_ctrl.theta_offset = 0.0f;  // Will be calibrated during d-axis alignment
    
    // Encoder offset calibration defaults
    foc_ctrl.calibration_enabled = 0;
    foc_ctrl.calib_sweep_angle = 0.0f;
    foc_ctrl.calib_best_offset = 0.0f;
    foc_ctrl.calib_best_encoder = 0.0f;
    foc_ctrl.calib_min_iq = 1000.0f;  // Initialize to large value
    foc_ctrl.calib_step_count = 0;
    foc_ctrl.calib_id_target = 0.4f;  // Default calibration current
    
    foc_ctrl.enabled = 0;
}

void FOC_Enable(void)
{
    PID_Reset(&foc_ctrl.pid_id);
    PID_Reset(&foc_ctrl.pid_iq);
    foc_ctrl.omega_slip = 0.0f;
    foc_ctrl.theta_slip = 0.0f;
    foc_ctrl.rotor_flux_wb = foc_ctrl.Lm * foc_ctrl.Id_ref;
    foc_ctrl.i_clarke.alpha = 0.0f;
    foc_ctrl.i_clarke.beta = 0.0f;
    foc_ctrl.i_park.d = 0.0f;
    foc_ctrl.i_park.q = 0.0f;
    foc_ctrl.enabled = 1;
}

void FOC_Disable(void)
{
    foc_ctrl.enabled = 0;
}

void FOC_SetIdReference(float Id_ref)
{
    // Limit to reasonable values (0 to 3A for typical small motors)
    if (Id_ref < 0.0f) Id_ref = 0.0f;
    if (Id_ref > 3.0f) Id_ref = 3.0f;
    foc_ctrl.Id_ref = Id_ref;
}

void FOC_SetIqReference(float Iq_ref)
{
    // Limit to reasonable values (-5 to 5A)
    if (Iq_ref < -5.0f) Iq_ref = -5.0f;
    if (Iq_ref > 5.0f) Iq_ref = 5.0f;
    foc_ctrl.Iq_ref = Iq_ref;
}

Clarke_Out_t FOC_Update(CurSense_Data_t currents, float theta, float ts)
{
    Clarke_Out_t voltage_ref = {0.0f, 0.0f};
    
    if (!foc_ctrl.enabled) {
        return voltage_ref;
    }
    
    // Update Vbus for voltage normalization
    foc_ctrl.Vbus = currents.Vbus;
    
    // Check if Vbus is sufficient
    if (foc_ctrl.Vbus < foc_ctrl.Vbus_min) {
        // Undervoltage - disable control
        return voltage_ref;
    }
    
    // Select angle based on mode
    float theta_sync;
    float theta_park;  // Angle for Park transform (d-q measurement frame)
    
    if (foc_ctrl.mode == FOC_MODE_OPEN_LOOP) {
        // Use imposed angle in open-loop mode
        theta_sync = WrapAngle0To2Pi(foc_ctrl.openloop_angle);
        theta_park = theta_sync;  // Same angle for both transforms in open-loop
    } else {
        // IRFOC: Park uses rotor flux angle, Inverse Park uses synchronous angle
        // theta_park aligns d-axis with rotor flux (for current measurement)
        theta_park = WrapAngle0To2Pi(theta + foc_ctrl.theta_offset);
        
        // theta_sync is stator field angle (rotor flux + slip)
        theta_sync = WrapAngle0To2Pi(theta_park + foc_ctrl.theta_slip);
        
        if (foc_ctrl.slip_comp_enabled == 0U) {
            foc_ctrl.omega_slip = 0.0f;
            foc_ctrl.theta_slip = 0.0f;
            theta_sync = theta_park;  // No slip - synchronous = rotor flux angle
        }
    }
    foc_ctrl.theta_sync = theta_sync;

    // Step 1: Clarke Transform (3-phase to 2-phase stationary)
    Clarke_Out_t i_alphabeta = Clarke_Transform(currents.Ia, currents.Ib, currents.Ic);
    foc_ctrl.i_clarke = i_alphabeta;  // Store for telemetry
    
    // Step 2: Park Transform (stationary to rotating d-q frame)
    // Use theta_park (rotor flux angle) for current measurement
    float sin_theta_park = 0.0f;
    float cos_theta_park = 1.0f;
    CORDIC_SinCos(theta_park, &sin_theta_park, &cos_theta_park);
    Park_Out_t i_dq = Park_Transform(i_alphabeta.alpha, i_alphabeta.beta, sin_theta_park, cos_theta_park);
    foc_ctrl.i_park = i_dq;  // Store for telemetry

    // IRFOC rotor flux model and slip speed update (current model)
    if (foc_ctrl.slip_comp_enabled != 0U) {
        float rr_over_lr = (foc_ctrl.Lr > 0.0f) ? (foc_ctrl.Rr / foc_ctrl.Lr) : 0.0f;
        float psi_r = foc_ctrl.rotor_flux_wb;

        // Rotor flux dynamics: dψr/dt = (Rr/Lr) * (Lm * Id - ψr)
        float psi_r_dot = rr_over_lr * (foc_ctrl.Lm * i_dq.d - psi_r);
        psi_r += psi_r_dot * ts;
        if (psi_r < foc_ctrl.min_flux_wb) {
            psi_r = foc_ctrl.min_flux_wb;
        }
        foc_ctrl.rotor_flux_wb = psi_r;

        // Slip speed: ω_slip = slip_gain * (Lm/ψr) * (Rr/Lr) * Iq
        foc_ctrl.omega_slip = foc_ctrl.slip_gain * (foc_ctrl.Lm / psi_r) * rr_over_lr * i_dq.q;
        foc_ctrl.theta_slip = WrapAngle0To2Pi(foc_ctrl.theta_slip + foc_ctrl.omega_slip * ts);
    }
    // Step 3: Current control - PI/PID loops in rotating frame
    float error_id = foc_ctrl.Id_ref - i_dq.d;
    float error_iq = foc_ctrl.Iq_ref - i_dq.q;
    
    // PID control for direct and quadrature currents
    foc_ctrl.Vd = PID_Update(&foc_ctrl.pid_id, error_id, ts);
    foc_ctrl.Vq = PID_Update(&foc_ctrl.pid_iq, error_iq, ts);
    
    // Step 4: Voltage saturation based on available Vbus
    // Maximum voltage magnitude in d-q frame (considering SVPWM capability)
    // Vmax = Vbus * (sqrt(3)/2) / sqrt(2) = Vbus * 0.612
    float V_max_available = foc_ctrl.Vbus * 0.612f;
    
    // Convert normalized voltages to volts
    float Vd_volts = foc_ctrl.Vd * V_max_available;
    float Vq_volts = foc_ctrl.Vq * V_max_available;
    
    // Circular limiting to maintain voltage magnitude within Vbus constraints
    float V_mag_sq = Vd_volts * Vd_volts + Vq_volts * Vq_volts;
    float V_max_sq = V_max_available * V_max_available;
    if (V_mag_sq > V_max_sq) {
        float V_mag = sqrtf(V_mag_sq);
        float scale = V_max_available / V_mag;
        Vd_volts *= scale;
        Vq_volts *= scale;
        foc_ctrl.Vd = Vd_volts / V_max_available;
        foc_ctrl.Vq = Vq_volts / V_max_available;
    }
    
    // Step 5: Inverse Park Transform (d-q to stationary alpha-beta)
    // Use theta_sync (rotor flux + slip) for voltage generation
    float sin_theta_sync = 0.0f;
    float cos_theta_sync = 1.0f;
    CORDIC_SinCos(theta_sync, &sin_theta_sync, &cos_theta_sync);
    
    // Normalize back to -1.0 to +1.0 range for PWM
    float Vd_norm = Vd_volts / V_max_available;
    float Vq_norm = Vq_volts / V_max_available;
    voltage_ref = Inv_Park_Transform(Vd_norm, Vq_norm, sin_theta_sync, cos_theta_sync);

    return voltage_ref;
}

FOC_Control_t* FOC_GetState(void)
{
    return &foc_ctrl;
}

void FOC_TunePID_Id(float Kp, float Ki, float Kd)
{
    foc_ctrl.pid_id.Kp = Kp;
    foc_ctrl.pid_id.Ki = Ki;
    foc_ctrl.pid_id.Kd = Kd;
}

void FOC_TunePID_Iq(float Kp, float Ki, float Kd)
{
    foc_ctrl.pid_iq.Kp = Kp;
    foc_ctrl.pid_iq.Ki = Ki;
    foc_ctrl.pid_iq.Kd = Kd;
}

void FOC_SetSlipCompensation(uint8_t enable, float slip_gain, float min_id)
{
    foc_ctrl.slip_comp_enabled = (enable != 0U) ? 1U : 0U;
    if (slip_gain < 0.0f) {
        slip_gain = 0.0f;
    }
    if (min_id < 0.01f) {
        min_id = 0.01f;
    }

    foc_ctrl.slip_gain = slip_gain;
    foc_ctrl.min_id_for_slip = min_id;
    foc_ctrl.min_flux_wb = foc_ctrl.Lm * min_id;
}

void FOC_SetMode(FOC_Mode_t mode)
{
    foc_ctrl.mode = mode;
    
    // Reset slip compensation when switching to open-loop
    if (mode == FOC_MODE_OPEN_LOOP) {
        foc_ctrl.omega_slip = 0.0f;
        foc_ctrl.theta_slip = 0.0f;
    }
}

FOC_Mode_t FOC_GetMode(void)
{
    return foc_ctrl.mode;
}

void FOC_StartOpenLoop(uint32_t align_time_ms, uint32_t ramp_time_ms, 
                       float target_freq_hz, float flux_id_a)
{
    // Store configuration (for future expansion if needed)
    (void)align_time_ms;    // Unused
    (void)ramp_time_ms;     // Unused
    (void)target_freq_hz;   // Unused
    (void)flux_id_a;        // Unused
    
    // DEPRECATED: Use FOC_StartCalibration() for calibrated startup instead
    // This function is kept for compatibility but not actively used
}

void FOC_StopOpenLoop(void)
{
    // DEPRECATED: Not used - call FOC_Disable() instead
    foc_ctrl.openloop_state = OPENLOOP_IDLE;
    foc_ctrl.openloop_angle = 0.0f;
    foc_ctrl.openloop_freq = 0.0f;
    foc_ctrl.Id_ref = 0.0f;
    foc_ctrl.Iq_ref = 0.0f;
}

void FOC_UpdateOpenLoop(float dt_sec, float theta_encoder)
{
    static float accumulated_time_sec = 0.0f;
    
    // Don't run open-loop state machine in closed-loop mode
    if (foc_ctrl.mode == FOC_MODE_CLOSED_LOOP) {
        return;
    }
    
    if (foc_ctrl.openloop_state == OPENLOOP_IDLE) {
        return;
    }
    
    // Accumulate time for millisecond tracking (called at 20kHz = 50us)
    accumulated_time_sec += dt_sec;
    if (accumulated_time_sec >= 0.001f) {  // Every 1ms
        uint32_t ms_increment = (uint32_t)(accumulated_time_sec * 1000.0f);
        foc_ctrl.openloop_time_ms += ms_increment;
        accumulated_time_sec -= (float)ms_increment / 1000.0f;
    }
    
    switch (foc_ctrl.openloop_state) {
        case OPENLOOP_COMPLETE:
            // Steady state: ready for closed-loop transition
            // Motor maintains current references set externally (via I/Q commands)
            // No angle ramping - user controls via UART commands
            break;
            
        case OPENLOOP_WAIT_INDEX:
            // Phase: Slow rotation waiting for encoder index pulse
            {
                // Maintain flux and torque currents set during FOC_StartCalibration()
                // foc_ctrl.Id_ref already set to flux_id_a
                // foc_ctrl.Iq_ref already set to 0.3f for rotation
                
                // Rotate at slow speed (2 Hz) to search for encoder index
                foc_ctrl.openloop_freq = 2.0f;
                
                // Integrate angle
                foc_ctrl.openloop_angle += 6.28318530718f * foc_ctrl.openloop_freq * dt_sec;
                foc_ctrl.openloop_angle = WrapAngle0To2Pi(foc_ctrl.openloop_angle);
            }
            break;
            
        case OPENLOOP_CALIBRATE:
            // Phase: Automatic encoder offset calibration
            // Algorithm: Sweep openloop_angle from 0 to 2π, find angle where |Iq| is minimum
            {
                foc_ctrl.Id_ref = foc_ctrl.calib_id_target;  // Magnetizing current
                foc_ctrl.Iq_ref = 0.0f;  // Zero torque command
                
                foc_ctrl.openloop_freq = 0.0f;  // Stand still (don't auto-increment)
                
                // Sweep configuration
                const uint32_t CALIB_STEPS = 36;  // 10° per step
                const uint32_t SETTLE_TIME_MS = 100;  // Time to wait at each angle
                
                // Wait for current to settle
                if (foc_ctrl.openloop_time_ms >= SETTLE_TIME_MS) {
                    // Sample Iq magnitude at this angle
                    float iq_abs = (foc_ctrl.i_park.q < 0.0f) ? -foc_ctrl.i_park.q : foc_ctrl.i_park.q;
                    
                    // Track minimum Iq and corresponding offset + encoder reading
                    if (iq_abs < foc_ctrl.calib_min_iq) {
                        foc_ctrl.calib_min_iq = iq_abs;
                        // Save both the voltage angle and the encoder reading at best alignment
                        foc_ctrl.calib_best_offset = foc_ctrl.calib_sweep_angle;
                        foc_ctrl.calib_best_encoder = theta_encoder;
                    }
                    
                    // Move to next angle
                    foc_ctrl.calib_step_count++;
                    
                    if (foc_ctrl.calib_step_count >= CALIB_STEPS) {
                        // Sweep complete - compute offset with inverted sign
                        // theta_offset = encoder_at_best - voltage_angle_best (INVERTED)
                        // In closed-loop: theta_sync = encoder + offset
                        // foc_ctrl.theta_offset = foc_ctrl.calib_best_encoder - foc_ctrl.calib_best_offset;
                        foc_ctrl.theta_offset = foc_ctrl.calib_best_offset - foc_ctrl.calib_best_encoder;  // Invert sign for correct alignment
                        foc_ctrl.openloop_angle = 0.0f;  // Set motor reference to 0 (neutral position)
                        
                        // Transition to COMPLETE state
                        foc_ctrl.openloop_state = OPENLOOP_COMPLETE;
                        foc_ctrl.calibration_enabled = 0;
                        foc_ctrl.openloop_time_ms = 0;
                    } else {
                        // Increment sweep angle (this is what varies during measurement)
                        foc_ctrl.calib_sweep_angle += 6.28318530718f / (float)CALIB_STEPS;
                        foc_ctrl.calib_sweep_angle = WrapAngle0To2Pi(foc_ctrl.calib_sweep_angle);
                        
                        // Apply this angle as the motor reference for sensing
                        foc_ctrl.openloop_angle = foc_ctrl.calib_sweep_angle;
                        
                        // Reset timer for next settling period
                        foc_ctrl.openloop_time_ms = 0;
                    }
                }
            }
            break;
            
        default:
            foc_ctrl.openloop_state = OPENLOOP_IDLE;
            break;
    }
}

OpenLoop_State_t FOC_GetOpenLoopState(void)
{
    return foc_ctrl.openloop_state;
}

void FOC_SetMotorParams(float Rs, float Rr, float Lm, float Ls, float Lr)
{
    if (Rs >= 0.0f) {
        foc_ctrl.Rs = Rs;
    }
    if (Rr >= 0.0f) {
        foc_ctrl.Rr = Rr;
    }
    if (Lm > 0.0f) {
        foc_ctrl.Lm = Lm;
    }
    if (Ls > 0.0f) {
        foc_ctrl.Ls = Ls;
    }
    if (Lr > 0.0f) {
        foc_ctrl.Lr = Lr;
    }

    foc_ctrl.min_flux_wb = foc_ctrl.Lm * foc_ctrl.min_id_for_slip;
    if (foc_ctrl.rotor_flux_wb < foc_ctrl.min_flux_wb) {
        foc_ctrl.rotor_flux_wb = foc_ctrl.min_flux_wb;
    }
}

void FOC_SetRotorFluxReference(float psi_ref_wb)
{
    if (psi_ref_wb < 0.0f) {
        psi_ref_wb = 0.0f;
    }
    if (foc_ctrl.Lm <= 0.0f) {
        return;
    }

    // Approximate Id_ref = psi_ref / Lm
    float id_ref = psi_ref_wb / foc_ctrl.Lm;
    FOC_SetIdReference(id_ref);
    if (psi_ref_wb > foc_ctrl.min_flux_wb) {
        foc_ctrl.rotor_flux_wb = psi_ref_wb;
    }
}

void FOC_SynchronizeSlipAngle(float theta_rotor)
{
    // Calculate angle difference to maintain continuity during mode switch
    // Before: theta_sync = openloop_angle (open-loop mode)
    // After:  theta_sync = theta_rotor + theta_offset + theta_slip (closed-loop mode)
    // To keep continuous: theta_slip = openloop_angle - theta_rotor - theta_offset
    
    float angle_diff = foc_ctrl.openloop_angle - theta_rotor - foc_ctrl.theta_offset;
    
    // Normalize to [0, 2π)
    foc_ctrl.theta_slip = WrapAngle0To2Pi(angle_diff);
}

void FOC_SetEncoderOffset(float offset_rad)
{
    foc_ctrl.theta_offset = WrapAngle0To2Pi(offset_rad);
}

float FOC_GetEncoderOffset(void)
{
    return foc_ctrl.theta_offset;
}

void FOC_StartCalibration(float flux_id_a)
{
    // Initialize calibration sequence
    foc_ctrl.calibration_enabled = 1;
    foc_ctrl.calib_id_target = flux_id_a;
    foc_ctrl.calib_sweep_angle = 0.0f;
    foc_ctrl.calib_best_offset = 0.0f;
    foc_ctrl.calib_best_encoder = 0.0f;
    foc_ctrl.calib_min_iq = 1000.0f;  // Reset to large value
    foc_ctrl.calib_step_count = 0;
    
    // Set motor control parameters
    foc_ctrl.mode = FOC_MODE_OPEN_LOOP;
    foc_ctrl.openloop_state = OPENLOOP_WAIT_INDEX;
    foc_ctrl.openloop_time_ms = 0;
    foc_ctrl.openloop_angle = 0.0f;
    foc_ctrl.openloop_freq = 2.0f;  // Slow rotation to find index
    foc_ctrl.Id_ref = flux_id_a;     // Directly set flux current for index search
    foc_ctrl.Iq_ref = 0.5f;          // Set torque for rotation
    
    // Reset PID controllers
    PID_Reset(&foc_ctrl.pid_id);
    PID_Reset(&foc_ctrl.pid_iq);
    
    // Initialize rotor flux
    foc_ctrl.rotor_flux_wb = foc_ctrl.Lm * flux_id_a;
}

void FOC_TriggerCalibration(void)
{
    // Transition from WAIT_INDEX to CALIBRATE state
    // Call this when encoder index is detected
    if (foc_ctrl.openloop_state == OPENLOOP_WAIT_INDEX && 
        foc_ctrl.calibration_enabled) {
        
        foc_ctrl.openloop_state = OPENLOOP_CALIBRATE;
        foc_ctrl.openloop_time_ms = 0;
        foc_ctrl.calib_sweep_angle = 0.0f;
        foc_ctrl.calib_step_count = 0;
        foc_ctrl.calib_min_iq = 1000.0f;
        foc_ctrl.theta_offset = 0.0f;  // Start sweep from zero
        
        // Maintain currents for calibration
        foc_ctrl.Id_ref = foc_ctrl.calib_id_target;  // Flux current
        foc_ctrl.Iq_ref = 0.0f;  // No torque during calibration
        foc_ctrl.openloop_freq = 0.0f;  // Stop rotation (standstill)
    }
}

uint8_t FOC_IsCalibrationComplete(void)
{
    return (foc_ctrl.calibration_enabled == 0 && 
            foc_ctrl.openloop_state == OPENLOOP_COMPLETE) ? 1U : 0U;
}

uint8_t FOC_CalibrateEncoderOffset(float theta_rotor, float id_target, uint32_t duration_ms)
{
    // This is a simplified version - actual calibration would be iterative
    // For now, perform basic alignment and let user fine-tune
    // DEPRECATED: Use FOC_StartCalibration() for automatic calibration
    
    (void)theta_rotor;    // Unused in this simplified version
    (void)duration_ms;    // Unused in this simplified version
    
    // Apply Id-only magnetizing current
    foc_ctrl.Id_ref = id_target;
    foc_ctrl.Iq_ref = 0.0f;
    foc_ctrl.enabled = 1;
    
    // Initial guess: encoder zero aligns with flux
    // User should monitor Iq and adjust offset until Iq ≈ 0
    foc_ctrl.theta_offset = 0.0f;
    
    // In a full implementation, this would:
    // 1. Sweep theta_offset from 0 to 2π
    // 2. Measure Iq at each angle
    // 3. Find angle where |Iq| is minimized
    // 4. That angle is where d-axis aligns with rotor flux
    
    // For now, return 1 to indicate initialization complete
    // User must manually adjust using FOC_SetEncoderOffset()
    
    return 1;
}
