#include "sine_op.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdint.h>


extern CORDIC_HandleTypeDef hcordic;

/**
 * LUT (Look-Up Table) storage for Sine values in Q15 format.
 * Size is determined by LUT_SIZE in header.
 */
static int16_t sine_lut_q15[LUT_SIZE];


/**
 * @brief Fast sine calculation using CORDIC hardware accelerator
 * @param angle: Angle in radians
 * @retval Sine value (-1.0 to 1.0)
 * @note CORDIC must be pre-configured in MX_CORDIC_Init() for sine calculation
 */
static inline float cordic_sin(float angle)
{
  const float TWO_PI = 6.28318530717958647692f;
  const float PI = 3.14159265358979323846f;
  
  // Normalize angle to [0, 2*PI]
  while (angle > TWO_PI) angle -= TWO_PI;
  while (angle < 0.0f) angle += TWO_PI;
  
  // Convert to [-PI, PI] range for CORDIC
  if (angle > PI) angle -= TWO_PI;
  
  // Convert radians to Q1.31: q31_value = angle * (2^31 / PI)
  int32_t angle_q31 = (int32_t)(angle * 683565275.5768f);
  
  int32_t sine_q31;
  HAL_CORDIC_CalculateZO(&hcordic, &angle_q31, &sine_q31, 1, HAL_MAX_DELAY);
  
  return (float)sine_q31 / 2147483648.0f;
}

/**
 * @brief Build sine lookup table using standard math library
 *        Generates one full cycle of sine wave scaled to Q15 format.
 */
void build_sine_lut(void)
{
    const float PI = 3.14159265358979323846f;
    const float TWO_PI = 2.0f * PI;

    for (uint32_t i = 0; i < LUT_SIZE; i++) {
        // Calculate angle for this index (0 to 2*PI)
        float angle_rad = (TWO_PI * (float)i) / (float)LUT_SIZE;
        
        // Calculate sine value (-1.0 to 1.0)
        float sine_val = cordic_sin(angle_rad);
        
        // Convert to Q15 fixed-point (-32767 to 32767)
        int32_t v = (int32_t)lrintf(sine_val * (float)Q15_MAX);
        
        // Clamp values to valid Q15 range
        if (v >  Q15_MAX) v =  Q15_MAX;
        if (v < -Q15_MAX) v = -Q15_MAX;
        
        sine_lut_q15[i] = (int16_t)v;
    }
}

/** 
 * @brief Get sine value from lookup table
 * @param phase: Phase angle in Q32 format (0 to 2^32-1 representing 0 to 2*PI)
 * @retval Sine value in Q15 format (-32767 to 32767)
 */
int16_t get_sine_q15(uint32_t phase)
{
    // Use the top LUT_BITS of the 32-bit phase accumulator to select LUT index
    uint16_t index = (uint16_t)(phase >> (32 - LUT_BITS)); 
    return sine_lut_q15[index];
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
    
    // Limit max compare value to ensure Low Side pulse is longer than deadtime
    // HRTIM Deadtime is configured to 200 ticks. We preserve 250 ticks to ensure partial switching.
    // This prevents the Low Side from disappearing completely ("0 state") due to deadtime insertion.
    // const uint32_t MIN_OFF_TIME = 250;
    // if (period_cycles > MIN_OFF_TIME) {
    //     if (compare_val > (period_cycles - MIN_OFF_TIME)) {
    //         compare_val = period_cycles - MIN_OFF_TIME;
    //     }
    // }

    return (uint32_t)compare_val;
}



