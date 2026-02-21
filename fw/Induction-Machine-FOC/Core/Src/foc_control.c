#include "foc_control.h"
#include <string.h>
#include <math.h>

static FOC_Control_t foc_ctrl = {0};
static const float FOC_TWO_PI = 6.28318530718f;

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
    // These values are starting points - tune for your motor
    foc_ctrl.pid_id.Kp = 2.0f;
    foc_ctrl.pid_id.Ki = 80.0f;
    foc_ctrl.pid_id.Kd = 0.0f;
    foc_ctrl.pid_id.output_min = -1.0f;
    foc_ctrl.pid_id.output_max = 1.0f;
    
    // PID parameters for Iq controller (torque control)
    foc_ctrl.pid_iq.Kp = 2.0f;
    foc_ctrl.pid_iq.Ki = 80.0f;
    foc_ctrl.pid_iq.Kd = 0.0f;
    foc_ctrl.pid_iq.output_min = -1.0f;
    foc_ctrl.pid_iq.output_max = 1.0f;
    
    // Maximum output voltage (normalized, typically sqrt(2/3) for SVPWM = 0.8165)
    foc_ctrl.max_voltage = 0.8165f;

    // Induction machine parameters (IRFOC model defaults)
    foc_ctrl.Rs = 0.039f;
    foc_ctrl.Rr = 0.031f;
    foc_ctrl.Lm = 1.36f;
    foc_ctrl.Ls = 1.5540f;
    foc_ctrl.Lr = 1.5540f;

    // IRFOC defaults (encoder-based rotor angle + slip compensation)
    foc_ctrl.slip_comp_enabled = 1U;
    foc_ctrl.slip_gain = 1.0f;
    foc_ctrl.min_id_for_slip = 0.2f;
    foc_ctrl.min_flux_wb = foc_ctrl.Lm * foc_ctrl.min_id_for_slip;
    foc_ctrl.rotor_flux_wb = foc_ctrl.Lm * foc_ctrl.Id_ref;
    foc_ctrl.omega_slip = 0.0f;
    foc_ctrl.theta_slip = 0.0f;
    foc_ctrl.theta_sync = 0.0f;
    
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
    if (foc_ctrl.mode == FOC_MODE_OPEN_LOOP) {
        // Use imposed angle in open-loop mode
        theta_sync = WrapAngle0To2Pi(foc_ctrl.openloop_angle);
    } else {
        // Use encoder angle + slip compensation in closed-loop mode
        theta_sync = WrapAngle0To2Pi(theta + foc_ctrl.theta_slip);
        if (foc_ctrl.slip_comp_enabled == 0U) {
            foc_ctrl.omega_slip = 0.0f;
            foc_ctrl.theta_slip = 0.0f;
            theta_sync = WrapAngle0To2Pi(theta);
        }
    }
    foc_ctrl.theta_sync = theta_sync;

    // Step 1: Clarke Transform (3-phase to 2-phase stationary)
    Clarke_Out_t i_alphabeta = Clarke_Transform(currents.Ia, currents.Ib, currents.Ic);
    foc_ctrl.i_clarke = i_alphabeta;  // Store for telemetry
    
    // Step 2: Park Transform (stationary to rotating d-q frame)
    float sin_theta = sinf(theta_sync);
    float cos_theta = cosf(theta_sync);
    Park_Out_t i_dq = Park_Transform(i_alphabeta.alpha, i_alphabeta.beta, sin_theta, cos_theta);
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
    float V_mag = sqrtf(Vd_volts * Vd_volts + Vq_volts * Vq_volts);
    if (V_mag > V_max_available) {
        float scale = V_max_available / V_mag;
        Vd_volts *= scale;
        Vq_volts *= scale;
        foc_ctrl.Vd = Vd_volts / V_max_available;
        foc_ctrl.Vq = Vq_volts / V_max_available;
    }
    
    // Step 5: Inverse Park Transform (d-q to stationary alpha-beta)
    // Normalize back to -1.0 to +1.0 range for PWM
    float Vd_norm = Vd_volts / V_max_available;
    float Vq_norm = Vq_volts / V_max_available;
    voltage_ref = Inv_Park_Transform(Vd_norm, Vq_norm, sin_theta, cos_theta);

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

// Open-loop startup configuration
static struct {
    uint32_t start_time_ms;
    uint32_t align_duration_ms;
    uint32_t ramp_duration_ms;
    float target_freq_hz;
    float flux_id_a;
} openloop_config = {0};

void FOC_StartOpenLoop(uint32_t align_time_ms, uint32_t ramp_time_ms, 
                       float target_freq_hz, float flux_id_a)
{
    // Store configuration
    openloop_config.align_duration_ms = align_time_ms;
    openloop_config.ramp_duration_ms = ramp_time_ms;
    openloop_config.target_freq_hz = target_freq_hz;
    openloop_config.flux_id_a = flux_id_a;
    
    // Initialize open-loop state
    foc_ctrl.mode = FOC_MODE_OPEN_LOOP;
    foc_ctrl.openloop_state = OPENLOOP_FLUX_ALIGN;
    foc_ctrl.openloop_angle = 0.0f;
    foc_ctrl.openloop_freq = 0.0f;
    foc_ctrl.openloop_time_ms = 0;
    
    // Set initial current references
    foc_ctrl.Id_ref = 0.0f;  // Start with low flux
    foc_ctrl.Iq_ref = 0.0f;  // Zero torque during alignment
    
    // Reset rotor flux estimate
    foc_ctrl.rotor_flux_wb = 0.0f;
    
    // Calculate acceleration for frequency ramp
    if (ramp_time_ms > 0) {
        foc_ctrl.openloop_accel = (target_freq_hz * 1000.0f) / (float)ramp_time_ms;  // Hz/s
    } else {
        foc_ctrl.openloop_accel = 10.0f;  // Default 10 Hz/s
    }
}

void FOC_StopOpenLoop(void)
{
    foc_ctrl.openloop_state = OPENLOOP_IDLE;
    foc_ctrl.openloop_angle = 0.0f;
    foc_ctrl.openloop_freq = 0.0f;
    foc_ctrl.Id_ref = 0.0f;
    foc_ctrl.Iq_ref = 0.0f;
}

void FOC_UpdateOpenLoop(float dt_sec)
{
    static float accumulated_time_sec = 0.0f;
    
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
        case OPENLOOP_FLUX_ALIGN:
            // Phase 1: Flux alignment (stationary rotor)
            // Gradually ramp up Id to align flux vector
            {
                float ramp_fraction = (float)foc_ctrl.openloop_time_ms / (float)openloop_config.align_duration_ms;
                if (ramp_fraction > 1.0f) ramp_fraction = 1.0f;
                
                // Ramp Id from 0 to target flux current
                foc_ctrl.Id_ref = openloop_config.flux_id_a * ramp_fraction;
                foc_ctrl.Iq_ref = 0.0f;
                foc_ctrl.openloop_angle = 0.0f;  // Keep at zero angle
                foc_ctrl.openloop_freq = 0.0f;
                
                // Transition to flux ramp after alignment time
                if (foc_ctrl.openloop_time_ms >= openloop_config.align_duration_ms) {
                    foc_ctrl.openloop_state = OPENLOOP_FLUX_RAMP;
                    foc_ctrl.openloop_time_ms = 0;
                }
            }
            break;
            
        case OPENLOOP_FLUX_RAMP:
            // Phase 2: Flux ramp (ensure full magnetization)
            // Keep Id at target, allow flux to build up
            {
                foc_ctrl.Id_ref = openloop_config.flux_id_a;
                foc_ctrl.Iq_ref = 0.0f;
                foc_ctrl.openloop_angle = 0.0f;
                foc_ctrl.openloop_freq = 0.0f;
                
                // Wait for flux to stabilize (typically 100-200 ms)
                uint32_t flux_ramp_time = 200;  // ms
                if (foc_ctrl.openloop_time_ms >= flux_ramp_time) {
                    foc_ctrl.openloop_state = OPENLOOP_ACCEL;
                    foc_ctrl.openloop_time_ms = 0;
                }
            }
            break;
            
        case OPENLOOP_ACCEL:
            // Phase 3: Frequency ramp (accelerate motor)
            {
                foc_ctrl.Id_ref = openloop_config.flux_id_a;
                
                // Ramp frequency linearly
                float elapsed_sec = (float)foc_ctrl.openloop_time_ms / 1000.0f;
                foc_ctrl.openloop_freq = foc_ctrl.openloop_accel * elapsed_sec;
                
                if (foc_ctrl.openloop_freq >= openloop_config.target_freq_hz) {
                    foc_ctrl.openloop_freq = openloop_config.target_freq_hz;
                    foc_ctrl.openloop_state = OPENLOOP_RUNNING;
                    foc_ctrl.openloop_time_ms = 0;
                }
                
                // Apply small Iq for startup torque (can be tuned)
                foc_ctrl.Iq_ref = 0.8f;  // Small torque current
                
                // Integrate angle: theta += 2*pi*freq*dt
                foc_ctrl.openloop_angle += 6.28318530718f * foc_ctrl.openloop_freq * dt_sec;
                foc_ctrl.openloop_angle = WrapAngle0To2Pi(foc_ctrl.openloop_angle);
            }
            break;
            
        case OPENLOOP_RUNNING:
            // Phase 4: Steady state open-loop
            {
                foc_ctrl.Id_ref = openloop_config.flux_id_a;
                foc_ctrl.Iq_ref = 0.8f;  // Maintain torque to avoid stall
                
                // Maintain constant frequency
                foc_ctrl.openloop_freq = openloop_config.target_freq_hz;
                
                // Integrate angle
                foc_ctrl.openloop_angle += 6.28318530718f * foc_ctrl.openloop_freq * dt_sec;
                foc_ctrl.openloop_angle = WrapAngle0To2Pi(foc_ctrl.openloop_angle);
                
                // After running for a while, signal ready for closed-loop
                if (foc_ctrl.openloop_time_ms >= 2000) {  // 2 seconds in steady state
                    foc_ctrl.openloop_state = OPENLOOP_COMPLETE;
                }
            }
            break;
            
        case OPENLOOP_COMPLETE:
            // Phase 5: Ready for transition to closed-loop
            {
                foc_ctrl.Id_ref = openloop_config.flux_id_a;
                foc_ctrl.Iq_ref = 0.8f;
                
                // Continue running at constant frequency
                foc_ctrl.openloop_freq = openloop_config.target_freq_hz;
                foc_ctrl.openloop_angle += 6.28318530718f * foc_ctrl.openloop_freq * dt_sec;
                foc_ctrl.openloop_angle = WrapAngle0To2Pi(foc_ctrl.openloop_angle);
                
                // Ready to switch to FOC_MODE_CLOSED_LOOP
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
