#include "foc_control.h"
#include "sine_op.h"
#include "foc_math.h"
#include "stm32g4xx_hal.h"
#include "svpwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

static void PWM_WriteCompareShadow(uint32_t cmp_a, uint32_t cmp_b, uint32_t cmp_c)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmp_c);
}


void open_loop_voltage_control(float Vd_ref, float Vq_ref, float angle_rad) {
    Clarke_Out_t valpha_beta;
    SVPWM_Output_t svpwm_output;
    
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    valpha_beta = Inv_Park_Transform(Vd_ref, Vq_ref, sin_theta, cos_theta);
    svpwm_output = SVPWM_Calculate(valpha_beta.alpha, valpha_beta.beta, 311);

    PWM_WriteCompareShadow(svpwm_output.duty_a, svpwm_output.duty_b, svpwm_output.duty_c);
}

void svpwm_test(void) {
    open_loop_voltage_control(0.6, 0.0, 0.0);
    HAL_Delay(500);

    for (int i= 0; i < 100; i++) {
        float mech_deg = (float)i * (360.0f / (float)100);
        float elec_deg = mech_deg * 2 /*MOTOR_POLE_PAIRS*/;
        open_loop_voltage_control(0.6, 0.0, DEG_TO_RAD(elec_deg));
        HAL_Delay(3);
    }
    open_loop_voltage_control(0.0f, 0.0f, 0.0f);
}

