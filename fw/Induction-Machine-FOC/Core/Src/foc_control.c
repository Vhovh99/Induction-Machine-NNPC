#include "foc_control.h"
#include "sine_op.h"
#include "foc_math.h"
#include <math.h>
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_uart.h"
#include "svpwm.h"
#include "main.h"
#include "stdio.h"
#include "encoder.h"

extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef hlpuart1;

extern Encoder_Handle_t encoder;

float theta_test = 0.0f; // remove later, just for testing open loop control

void PWM_WriteCompareShadow(float cmp_a, float cmp_b, float cmp_c, float cmp_trigger)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(cmp_a * PWM_PERIOD_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(cmp_b * PWM_PERIOD_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(cmp_c * PWM_PERIOD_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(cmp_trigger * PWM_PERIOD_TICKS));
}

void Motor_Init(const Motor_Parameters_t *params, Motor_Control_t *ctrl)
{   
    ctrl->state = MOTOR_STATE_IDLE;
    ctrl->theta_sl = 0.0f;
    ctrl->omega_sl = 0.0f;
    ctrl->omega_sl_filt = 0.0f;
    ctrl->omega_e = 0.0f;
    ctrl->theta_e = 0.0f;

    ctrl->psi_r = 0.0f;

    ctrl->v_dq.d = 0.0f;
    ctrl->v_dq.q = 0.0f;

    ctrl->flux_build_time = 0.0f;

    // Initialize PI controllers
    // Limits should be in VOLTS, not Amps!
    float max_voltage = FOC_PI_MAX_VOLTAGE;

    // PI tuned via pole-zero cancellation on stator transient model:
    //   Kp = sigma_Ls * omega_bw,  Ki = Rs * omega_bw
    float omega_bw = FOC_CURRENT_PI_BW;
    float sigma_Ls = params->sigma_Ls;

    ctrl->id_controller.Kp = sigma_Ls * omega_bw;  // 0.236*10 = 2.36
    ctrl->id_controller.Ki = params->Rs * omega_bw; // 52.43*10 = 524.3
    ctrl->id_controller.integral = 0.0f;
    ctrl->id_controller.out_min = -max_voltage;
    ctrl->id_controller.out_max = max_voltage;

    ctrl->iq_controller.Kp = sigma_Ls * omega_bw;
    ctrl->iq_controller.Ki = params->Rs * omega_bw;
    ctrl->iq_controller.integral = 0.0f;
    ctrl->iq_controller.out_min = -max_voltage;
    ctrl->iq_controller.out_max = max_voltage;

    // Speed PI controller
    // Output is iq_ref (A), so limits are the max torque current
    float iq_limit = params->iq_max;
    ctrl->speed_controller.Kp = FOC_SPEED_PI_KP;
    ctrl->speed_controller.Ki = FOC_SPEED_PI_KI;
    ctrl->speed_controller.integral = 0.0f;
    ctrl->speed_controller.out_min = -iq_limit;
    ctrl->speed_controller.out_max = iq_limit;

    ctrl->omega_ref_ramped = 0.0f;
    
}

static inline float pi_run(PI_t *pi, float err, float Ts)
{
    float proportional = pi->Kp * err;
    float output_preview = proportional + pi->integral;

    // Conditional integration (anti-windup): only integrate when output is
    // not saturated, or when the error would reduce the saturation.
    int sat_hi = (output_preview >= pi->out_max);
    int sat_lo = (output_preview <= pi->out_min);

    if ((!sat_hi || err < 0.0f) && (!sat_lo || err > 0.0f)) {
        pi->integral += (pi->Ki * err) * Ts;
    }

    float out = proportional + pi->integral;

    // output clamp
    if (out > pi->out_max) out = pi->out_max;
    if (out < pi->out_min) out = pi->out_min;

    return out;
}

void open_loop_voltage_control(float Vd_ref, float Vq_ref, float angle_rad) {
    Clarke_Out_t valpha_beta;
    SVPWM_Output_t svpwm_output;
    
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    valpha_beta = Inv_Park_Transform(Vd_ref, Vq_ref, sin_theta, cos_theta);
    svpwm_output = SVPWM_Calculate(valpha_beta.alpha, valpha_beta.beta, 311);

    PWM_WriteCompareShadow(svpwm_output.duty_a, svpwm_output.duty_b, svpwm_output.duty_c, svpwm_output.trigger_point);
    // char buffer[64];
    // int len = snprintf(buffer, sizeof(buffer), "%.0f,%.0f,%.0f,%.0f\r\n", 
    //                    svpwm_output.duty_a * PWM_PERIOD_TICKS, svpwm_output.duty_b * PWM_PERIOD_TICKS, svpwm_output.duty_c * PWM_PERIOD_TICKS, svpwm_output.trigger_point * PWM_PERIOD_TICKS);
    // HAL_UART_Transmit(&hlpuart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

void svpwm_test(void) {

    float max_freq = 5.0f;        // Target frequency in Hz (keep it low for open loop!)
    float voltage_amplitude = 40.0f; // Start with small voltage  

    float theta = 0.0f;
    float dt = 0.001f; // 1ms delay = 1kHz loop approximation
    float d_theta = 2.0f * 3.14159f * max_freq * dt; 

    // Rotate for 5 seconds
    for (int i= 0; i < 5000; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        theta += d_theta;
        if(theta > 6.2831f) theta -= 6.2831f;
        
        // In open loop, we just apply a rotating voltage vector.
        // We put all magnitude in D (or Q, doesn't matter, it's just a vector magnitude)
        // effectively Vd=3V, Vq=0V in a frame that is ROTATING at 5Hz.
        open_loop_voltage_control(voltage_amplitude, 0.0f, theta);
        Encoder_Update(&encoder, dt);
        theta_test = Encoder_GetMechanicalAngleRad(&encoder); 
        // char buf [64];
        // int len =  sprintf(buf, "%.2f\r\n", RAD_TO_DEG(temp));
        // HAL_UART_Transmit(&hlpuart1, (uint8_t*)buf, len, 1);
        // HAL_Delay(1); // Wait 1ms
        
        if (!(HAL_ADC_GetState(&hadc2) & HAL_ADC_STATE_REG_BUSY))
        {
            HAL_ADC_Start_IT(&hadc2);
        }
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    }

    // 4. Stop
    open_loop_voltage_control(0.0f, 0.0f, 0.0f);

}

static inline void Flux_Slip_Update(Motor_Control_t *control, const Motor_Parameters_t *params,
                                     float id_ref, float iq_for_slip) {

    // Avoid division by zero
    const float PSI_MIN = FOC_PSI_MIN;

    // Flux estimation uses REFERENCE id (feedforward)
    // d(psi_r)/dt = (Lm/Tr) * id_ref - (1/Tr) * psi_r
    float dpsi = (params->Lm / params->Tr) * id_ref - (1.0f / params->Tr) * control->psi_r;
    control->psi_r += dpsi * params->Ts;
    
    if (control->psi_r < PSI_MIN) {
        control->psi_r = PSI_MIN;
    }

    // omega_sl = (Lm / Tr) * i_q / psi_r
    control->omega_sl = (params->Lm / params->Tr) * (iq_for_slip / control->psi_r);

    // Low-pass filter omega_sl before integration — prevents noisy iq_ref
    // from corrupting theta_e (effect grows with electrical speed)
    control->omega_sl_filt = FOC_OMEGA_SL_LPF_ALPHA * control->omega_sl_filt
                          + (1.0f - FOC_OMEGA_SL_LPF_ALPHA) * control->omega_sl;

    // theta_sl = theta_sl + omega_sl_filt * Ts
    control->theta_sl = WrapAngle0To2Pi(control->theta_sl + control->omega_sl_filt * params->Ts);

}


void FOC_Control_Loop(Motor_Control_t *ctrl, const Motor_Parameters_t *params,
                      float ia, float ib,
                      float id_ref_cmnd, float omega_ref,
                      float theta_m, float omega_m, float vbus, SVPWM_Output_t *out_svpwm) {
    
    Clarke_Out_t clarke;
    Park_Out_t   park;
    SVPWM_Output_t svpwm_output;

    ctrl->theta_m = theta_m;
    ctrl->omega_m = omega_m;

    /* Compute electrical angular velocity and integrate to get theta_e for Park.
     * omega_sl_filt is from the previous cycle (updated later in Flux_Slip_Update),
     * which is a valid one-sample delay at 20 kHz. */
    ctrl->omega_e = omega_m * (float)params->pole_pairs + ctrl->omega_sl;

    /* PLL correction: pull the integrator toward the encoder+slip angle
     * to eliminate drift, while keeping the smoothness of integration.
     * Error is wrapped to [-pi, pi] to handle the 0/2pi boundary. */
    float theta_e_encoder = WrapAngle0To2Pi(ctrl->theta_m * (float)params->pole_pairs + ctrl->theta_sl);
    float theta_err = theta_e_encoder - ctrl->theta_e;
    if (theta_err >  3.14159265f) theta_err -= 6.28318530f;
    if (theta_err < -3.14159265f) theta_err += 6.28318530f;
    ctrl->omega_e += FOC_THETA_CORR_GAIN * theta_err;

    ctrl->theta_e = WrapAngle0To2Pi(ctrl->theta_e + ctrl->omega_e * params->Ts);

    clarke = Clarke_Transform(ia, ib);

    float sin_theta, cos_theta;
    pre_calc_sin_cos(ctrl->theta_e, &sin_theta, &cos_theta);
    park = Park_Transform(clarke.alpha, clarke.beta, sin_theta, cos_theta);
    
    ctrl->i_dq = park;

    float id_ref = 0.0f, iq_ref = 0.0f;

    switch (ctrl->state) {
        case MOTOR_STATE_IDLE:
            ctrl->psi_r = 0.0f;
            ctrl->theta_sl = 0.0f;
            ctrl->flux_build_time = 0.0f;

            ctrl->v_dq.d = 0.0f;
            ctrl->v_dq.q = 0.0f;

            ctrl->id_controller.integral = 0.0f;
            ctrl->iq_controller.integral = 0.0f;
            ctrl->speed_controller.integral = 0.0f;
            ctrl->omega_ref_ramped = 0.0f;
            ctrl->omega_sl_filt = 0.0f;
            ctrl->omega_e = 0.0f;
            ctrl->theta_e = 0.0f;

            ctrl->state = MOTOR_STATE_FLUX_BUILD;
            break;
        case MOTOR_STATE_FLUX_BUILD:
            // Build flux with i_d, keep torque current 0
            id_ref = id_ref_cmnd;
            iq_ref = 0.0f;
            ctrl->flux_build_time += params->Ts;
            Flux_Slip_Update(ctrl, params, id_ref, 0);
            if (ctrl->flux_build_time >= FOC_FLUX_BUILD_TIME) {
                ctrl->state = MOTOR_STATE_RUNNING;
            }
            break;
        case MOTOR_STATE_RUNNING:
        default:
            id_ref = id_ref_cmnd;

            // Speed reference rate limiter
            float ramp_step = FOC_SPEED_RAMP_RATE * params->Ts;
            float diff = omega_ref - ctrl->omega_ref_ramped;
            if (diff > ramp_step) diff = ramp_step;
            if (diff < -ramp_step) diff = -ramp_step;
            ctrl->omega_ref_ramped += diff;

            // Outer speed PI: error → iq_ref
            float speed_err = ctrl->omega_ref_ramped - omega_m;
            iq_ref = pi_run(&ctrl->speed_controller, speed_err, params->Ts);

            Flux_Slip_Update(ctrl, params, id_ref, iq_ref);
            break;

    }

    float err_d = id_ref - ctrl->i_dq.d;
    float err_q = iq_ref - ctrl->i_dq.q;

    ctrl->v_dq.d = pi_run(&ctrl->id_controller, err_d, params->Ts);
    ctrl->v_dq.q = pi_run(&ctrl->iq_controller, err_q, params->Ts);

    // Feedforward decoupling — reuse omega_e already computed above
    ctrl->v_dq.d += -ctrl->omega_e * params->sigma_Ls * iq_ref;
    ctrl->v_dq.q +=  ctrl->omega_e * params->sigma_Ls * id_ref;

    Park_Out_t v_dq = ctrl->v_dq;

    // Voltage limiter with d-axis priority:
    // Clamp Vd first (guarantees flux current), then use the remaining
    // circle budget for Vq. This prevents q-axis saturation from
    // stealing d-axis voltage and collapsing the rotor flux.
    float v_max = vbus * FOC_ONE_BY_SQRT3;
    if (v_dq.d >  v_max) v_dq.d =  v_max;
    if (v_dq.d < -v_max) v_dq.d = -v_max;
    float vq_budget = sqrtf(v_max * v_max - v_dq.d * v_dq.d);
    if (v_dq.q >  vq_budget) v_dq.q =  vq_budget;
    if (v_dq.q < -vq_budget) v_dq.q = -vq_budget;

    Clarke_Out_t valpha_beta = Inv_Park_Transform(v_dq.d, v_dq.q, sin_theta, cos_theta);

    // Electromagnetic torque: Te = (3/2) * p * (Lm/Lr) * psi_r * iq
    ctrl->torque_e = 1.5f * (float)params->pole_pairs
                   * (params->Lm / params->Lr)
                   * ctrl->psi_r
                   * ctrl->i_dq.q;

    svpwm_output = SVPWM_Calculate(valpha_beta.alpha, valpha_beta.beta, vbus);

    if (out_svpwm != NULL) {
        *out_svpwm = svpwm_output;
    }
}