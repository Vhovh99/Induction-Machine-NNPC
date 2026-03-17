#include "sine_op.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdint.h>


extern CORDIC_HandleTypeDef hcordic;



/**
 * @brief Fast sine calculation using CORDIC hardware accelerator
 * @param angle: Angle in radians
 * @retval Sine value (-1.0 to 1.0)
 */
float cordic_sin(float angle)
{
    const float TWO_PI = 6.28318530718f;
    const float PI = 3.14159265359f;
    const float Q31_PER_PI = 683565275.5768f;
    const float Q31_INV = 1.0f / 2147483648.0f;

    // Normalize angle to [-PI, PI] for CORDIC input.
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0.0f) {
        angle += TWO_PI;
    }
    if (angle > PI) {
        angle -= TWO_PI;
    }

    int32_t angle_q31 = (int32_t)(angle * Q31_PER_PI);

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

    int32_t sine_q31 = (int32_t)CORDIC->RDATA;
    (void)CORDIC->RDATA; // Discard cosine result.

    return (float)sine_q31 * Q31_INV;
}

float cordic_cos(float angle)
{
    const float PI_2 = 1.57079632679f; // PI/2
    return cordic_sin(angle + PI_2);
}

void pre_calc_sin_cos(float theta, float *sin_theta, float *cos_theta) 
{
    *sin_theta = cordic_sin(theta);
    *cos_theta = cordic_cos(theta);
}


/**
 * @brief Convert sine value to PWM compare register value
 * 
 * @param s_q15: Sine value in Q15 format (-32767 to 32767)
 * @param period_cycles: PWM timer period in cycles
 * @param modulation_q15: Modulation index in Q15 format (0 to 32767)
 * @return uint32_t: Compare register value for PWM timer
 */
uint32_t sine_to_cmp(int16_t s_q15, uint32_t period_cycles, int32_t modulation_q15)
{
    // 1. Scale sine by modulation index
    //    s_q15 [-32767, 32767] * m_q15 [0, 32767] = Q30 result
    int32_t scaled_sine = (int32_t)s_q15 * (int32_t)modulation_q15;
    
    //    Convert back to Q15
    scaled_sine >>= 15; 

    // 2. Convert bipolar sine (-1..1) to unipolar duty cycle (0..1)
    //    Duty = 0.5 + 0.5 * sine
    //    In Q15: Duty = 16384 + (scaled_sine / 2)
    int32_t duty_q15 = 16384 + (scaled_sine >> 1); // 16384 is 0.5 in Q15

    // 3. Clamp duty cycle to safe limits
    if (duty_q15 < 0) duty_q15 = 0;
    if (duty_q15 > 32767) duty_q15 = 32767;

    // 4. Calculate Compare Value: Period * Duty
    //    Use 64-bit to prevent overflow during multiply
    uint64_t compare_val = (uint64_t)duty_q15 * (uint64_t)period_cycles;
    
    //    Shift back from Q15 to integer domain
    compare_val >>= 15;
    
    return (uint32_t)compare_val;
}



