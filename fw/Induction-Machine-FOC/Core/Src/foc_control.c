#include "foc_control.h"
#include "sine_op.h"
#include "foc_math.h"
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
    //ctrl->theta_e = 0.0f;

    ctrl->psi_r = 0.0f;

    ctrl->v_dq.d = 0.0f;
    ctrl->v_dq.q = 0.0f;

    ctrl->flux_build_time = 0.0f;

    // Initialize PI controllers
    // Limits should be in VOLTS, not Amps!
    float max_voltage = 50.0f; // Limit raised for back-EMF overhead 

    ctrl->id_controller.Kp = 0.5f; // Increased Kp for 55 ohm stator
    ctrl->id_controller.Ki = 50.0f; // Increased Ki for 55 ohm stator
    ctrl->id_controller.integral = 0.0f;
    ctrl->id_controller.out_min = -max_voltage;
    ctrl->id_controller.out_max = max_voltage;

    ctrl->iq_controller.Kp = 0.5f;
    ctrl->iq_controller.Ki = 50.0f;
    ctrl->iq_controller.integral = 0.0f;
    ctrl->iq_controller.out_min = -max_voltage;
    ctrl->iq_controller.out_max = max_voltage;
    
}

static inline float pi_run(PI_t *pi, float err, float Ts)
{
    // integrator
    pi->integral += (pi->Ki * err) * Ts;

    // anti-windup clamp integrator (simple)
    if (pi->integral > pi->out_max) pi->integral = pi->out_max;
    if (pi->integral < pi->out_min) pi->integral = pi->out_min;

    float out = pi->Kp * err + pi->integral;

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
    // open_loop_voltage_control(0.0 * 311, 0.0 * 311, 0.0);
    // HAL_Delay(500);

    // for (int i= 0; i < 500; i++) {
    //     float mech_deg = (float)i * (360.0f / (float)500);
    //     float elec_deg = mech_deg * 2 /*MOTOR_POLE_PAIRS*/;
    //     open_loop_voltage_control(0.0 * 311, 0.0 * 311, DEG_TO_RAD(elec_deg));
    //     HAL_Delay(3);
    // }
    // open_loop_voltage_control(0.0f, 0.0f, 0.0f);

    float v_bus = 311.0f;          // Your DC Bus Voltage (e.g. 24V for testing)
    float max_freq = 5.0f;        // Target frequency in Hz (keep it low for open loop!)
    float voltage_amplitude = 40.0f; // Start with small voltage  

    // open_loop_voltage_control(voltage_amplitude, 0.0f, 0.0f);
    // HAL_Delay(1000); 

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

static inline void Flux_Slip_Update(Motor_Control_t *control, const Motor_Parameters_t *params) {

    // Avoid division by zero
    const float PSI_MIN = 1e-3f;

    // d(psi_r)/dt = (Lm/Tr) * i_d - (1/Tr) * psi_r
    float dpsi = (params->Lm / params->Tr) * control->i_dq.d - (1.0f / params->Tr) * control->psi_r;
    control->psi_r += dpsi * params->Ts;
    
    if (control->psi_r < PSI_MIN) {
        control->psi_r = PSI_MIN;
    }

    // omega_sl = (Lm / Tr) * i_q / psi_r
    control->omega_sl = (params->Lm / params->Tr) * (control->i_dq.q / control->psi_r);

    // theta_sl = theta_sl + omega_sl * Ts
    control->theta_sl = WrapAngle0To2Pi(control->theta_sl + control->omega_sl * params->Ts);

}


void FOC_Control_Loop(Motor_Control_t *ctrl, const Motor_Parameters_t *params,
                      float ia, float ib,
                      float id_ref_cmnd, float iq_ref_cmnd,
                      float theta_m, float vbus, SVPWM_Output_t *out_svpwm) {
    
    Clarke_Out_t clarke;
    Park_Out_t   park;
    SVPWM_Output_t svpwm_output;

    ctrl->theta_m = theta_m;
    ctrl->theta_e = WrapAngle0To2Pi(ctrl->theta_m * (float)params->pole_pairs + ctrl->theta_sl);
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

            ctrl->state = MOTOR_STATE_FLUX_BUILD;
            break;
        case MOTOR_STATE_FLUX_BUILD:
            // Build flux with i_d, keep torque current 0
            id_ref = id_ref_cmnd;
            iq_ref = 0.0f;
            ctrl->flux_build_time += params->Ts;
            Flux_Slip_Update(ctrl, params);
            if (ctrl->flux_build_time >= 0.2f) { // 200 ms to build flux
                ctrl->state = MOTOR_STATE_RUNNING;
            }
            break;
        case MOTOR_STATE_RUNNING:
        default:
            id_ref = id_ref_cmnd;
            iq_ref = iq_ref_cmnd;
            Flux_Slip_Update(ctrl, params);
            break;

    }

    float err_d = id_ref - ctrl->i_dq.d;
    float err_q = iq_ref - ctrl->i_dq.q;

    ctrl->v_dq.d = pi_run(&ctrl->id_controller, err_d, params->Ts);
    ctrl->v_dq.q = pi_run(&ctrl->iq_controller, err_q, params->Ts);

    Park_Out_t v_dq = ctrl->v_dq;
    Clarke_Out_t valpha_beta = Inv_Park_Transform(v_dq.d, v_dq.q, sin_theta, cos_theta);

    svpwm_output = SVPWM_Calculate(valpha_beta.alpha, valpha_beta.beta, vbus);

    if (out_svpwm != NULL) {
        *out_svpwm = svpwm_output;
    }
}