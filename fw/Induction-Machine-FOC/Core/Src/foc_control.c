#include "foc_control.h"
#include "sine_op.h"
#include "foc_math.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_uart.h"
#include "svpwm.h"
#include "main.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef hlpuart1;

static void PWM_WriteCompareShadow(float cmp_a, float cmp_b, float cmp_c, float cmp_trigger)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(cmp_a * PWM_PERIOD_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(cmp_b * PWM_PERIOD_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(cmp_c * PWM_PERIOD_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)(cmp_trigger * PWM_PERIOD_TICKS));
}


void open_loop_voltage_control(float Vd_ref, float Vq_ref, float angle_rad) {
    Clarke_Out_t valpha_beta;
    SVPWM_Output_t svpwm_output;
    
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    valpha_beta = Inv_Park_Transform(Vd_ref, Vq_ref, sin_theta, cos_theta);
    svpwm_output = SVPWM_Calculate(valpha_beta.alpha, valpha_beta.beta, 311);

    PWM_WriteCompareShadow(svpwm_output.duty_a, svpwm_output.duty_b, svpwm_output.duty_c, svpwm_output.trigger_point);
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "%.0f,%.0f,%.0f,%.0f\r\n", 
                       svpwm_output.duty_a * PWM_PERIOD_TICKS, svpwm_output.duty_b * PWM_PERIOD_TICKS, svpwm_output.duty_c * PWM_PERIOD_TICKS, svpwm_output.trigger_point * PWM_PERIOD_TICKS);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

void svpwm_test(void) {
    open_loop_voltage_control(0.6 * 311, 0.0 * 311, 0.0);
    HAL_Delay(500);

    for (int i= 0; i < 500; i++) {
        float mech_deg = (float)i * (360.0f / (float)500);
        float elec_deg = mech_deg * 2 /*MOTOR_POLE_PAIRS*/;
        open_loop_voltage_control(0.6 * 311, 0.0 * 311, DEG_TO_RAD(elec_deg));
        HAL_Delay(3);
    }
    open_loop_voltage_control(0.0f, 0.0f, 0.0f);
}

