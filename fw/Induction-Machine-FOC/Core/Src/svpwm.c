#include "svpwm.h"
#include <math.h>
#include <string.h>
#include "main.h"

// Constants for SVPWM calculation
#define ONE_BY_SQRT3 0.577350269f
#define TWO_BY_SQRT3 1.154700538f

extern TIM_HandleTypeDef htim1;

static SVPWM_Config_t config = {0};

void SVPWM_Init(SVPWM_Config_t *cfg)
{
    memcpy(&config, cfg, sizeof(SVPWM_Config_t));
    
    // Ensure reasonable defaults
    if (config.min_duty_window < 0.05f) {
        config.min_duty_window = 0.10f;  // 10% minimum duty for valid shunt
    }
    if (config.trigger_offset < 0.01f) {
        config.trigger_offset = 0.05f;   // 5% offset from duty edge
    }
 
    if (config.pwm_period_ticks <= 4250) {
        config.pwm_period_ticks = 4250;  // Default to 20 kHz with 170 MHz timer clock
    }
}

SVPWM_Sector_t SVPWM_GetSector(float v_alpha, float v_beta)
{
    // Calculate sector based on alpha-beta voltage angle
    // Sector boundaries at 0°, 60°, 120°, 180°, 240°, 300°
    
    // Determine sector using Clarke components
    // This is more efficient than calculating atan2
    
    float sqrt3_beta = 1.732050808f * v_beta;  // sqrt(3) * v_beta
    
    uint8_t A = (v_beta > 0.0f) ? 1 : 0;
    uint8_t B = (sqrt3_beta > v_alpha) ? 1 : 0;
    uint8_t C = (sqrt3_beta > -v_alpha) ? 1 : 0;
    
    // Sector determination from ABC bits
    uint8_t N = 4 * C + 2 * B + A;
    
    // Map N to sector (1-6)
    const SVPWM_Sector_t sector_map[8] = {
        SVPWM_SECTOR_6,  // N=0
        SVPWM_SECTOR_1,  // N=1
        SVPWM_SECTOR_2,  // N=2
        SVPWM_SECTOR_2,  // N=3
        SVPWM_SECTOR_5,  // N=4
        SVPWM_SECTOR_6,  // N=5
        SVPWM_SECTOR_4,  // N=6
        SVPWM_SECTOR_3   // N=7
    };
    
    return sector_map[N];
}



SVPWM_Output_t SVPWM_Calculate(float v_alpha, float v_beta, float v_bus)
{
    SVPWM_Output_t output = {0};

    // 1. Normalize voltages by vbus
    float alpha_norm, beta_norm;
    if (v_bus > 0.1f) {
        alpha_norm = v_alpha / v_bus;
        beta_norm = v_beta / v_bus;
    } else {
        alpha_norm = 0.0f;
        beta_norm = 0.0f;
    }

    // 2. Sector determination
    uint8_t sector;
    if (beta_norm >= 0.0f) {
        if (alpha_norm >= 0.0f) {
            sector = (ONE_BY_SQRT3 * beta_norm > alpha_norm) ? 2 : 1;
        } else {
            sector = (-ONE_BY_SQRT3 * beta_norm > alpha_norm) ? 3 : 2;
        }
    } else {
        if (alpha_norm >= 0.0f) {
            sector = (-ONE_BY_SQRT3 * beta_norm > alpha_norm) ? 5 : 6;
        } else {
            sector = (ONE_BY_SQRT3 * beta_norm > alpha_norm) ? 4 : 5;
        }
    }
    output.sector = (SVPWM_Sector_t)sector;

    // 3. Calculate active vector times (normalized period = 1.0)
    float t1, t2;
    float pwm_period = 1.0f;

    switch(sector) {
        case 1:
            t1 = (alpha_norm - ONE_BY_SQRT3 * beta_norm) * pwm_period;
            t2 = (TWO_BY_SQRT3 * beta_norm * pwm_period);
            output.duty_a = (pwm_period + t1 + t2) / 2.0f;
            output.duty_b = output.duty_a - t1;
            output.duty_c = output.duty_b - t2;
            break;

        case 2:
            t1 = (alpha_norm + ONE_BY_SQRT3 * beta_norm) * pwm_period;
            t2 = (-alpha_norm + ONE_BY_SQRT3 * beta_norm) * pwm_period;
            output.duty_b = (pwm_period + t1 + t2) / 2.0f;
            output.duty_a = output.duty_b - t2;
            output.duty_c = output.duty_a - t1;
            break;

        case 3:
            t1 = (TWO_BY_SQRT3 * beta_norm * pwm_period);
            t2 = (-alpha_norm - ONE_BY_SQRT3 * beta_norm) * pwm_period;
            output.duty_b = (pwm_period + t1 + t2) / 2.0f;
            output.duty_c = output.duty_b - t1;
            output.duty_a = output.duty_c - t2;
            break;

        case 4:
            t1 = (-alpha_norm + ONE_BY_SQRT3 * beta_norm) * pwm_period;
            t2 = (-TWO_BY_SQRT3 * beta_norm * pwm_period);
            output.duty_c = (pwm_period + t1 + t2) / 2.0f;
            output.duty_b = output.duty_c - t2;
            output.duty_a = output.duty_b - t1;
            break;

        case 5:
            t1 = (-alpha_norm - ONE_BY_SQRT3 * beta_norm) * pwm_period;
            t2 = (alpha_norm - ONE_BY_SQRT3 * beta_norm) * pwm_period;
            output.duty_c = (pwm_period + t1 + t2) / 2.0f;
            output.duty_a = output.duty_c - t1;
            output.duty_b = output.duty_a - t2;
            break;

        case 6:
            t1 = (-TWO_BY_SQRT3 * beta_norm * pwm_period);
            t2 = (alpha_norm + ONE_BY_SQRT3 * beta_norm) * pwm_period;
            output.duty_a = (pwm_period + t1 + t2) / 2.0f;
            output.duty_c = output.duty_a - t2;
            output.duty_b = output.duty_c - t1;
            break;
            
        default:
            output.duty_a = pwm_period * 0.5f;
            output.duty_b = pwm_period * 0.5f;
            output.duty_c = pwm_period * 0.5f;
            break;
    }
       
    if (output.duty_a > 0.95f) output.duty_a = 0.95f;
    if (output.duty_a < 0.05f) output.duty_a = 0.05f;
    if (output.duty_b > 0.95f) output.duty_b = 0.95f;
    if (output.duty_b < 0.05f) output.duty_b = 0.05f;
    if (output.duty_c > 0.95f) output.duty_c = 0.95f;
    if (output.duty_c < 0.05f) output.duty_c = 0.05f;

    // Select valid shunts
    output.valid = SVPWM_SelectShunts(output.sector, output.duty_a, output.duty_b, output.duty_c,
                                      &output.shunt1, &output.shunt2);
    
    // Calculate trigger point
    if (output.valid) {
        output.trigger_point = SVPWM_CalculateTriggerPoint(output.sector, 
                                                            output.duty_a, output.duty_b, output.duty_c,
                                                            output.shunt1, output.shunt2);
    } else {
        // Fallback: trigger near peak where all low-sides are ON
        output.trigger_point = 0.998f;
    }
    if (output.trigger_point > 0.998f) output.trigger_point = 0.998f;
    if (output.trigger_point < 0.05f) output.trigger_point = 0.05f;

    return output;
}

uint8_t SVPWM_SelectShunts(SVPWM_Sector_t sector, float duty_a, float duty_b, float duty_c,
                           Phase_t *shunt1, Phase_t *shunt2)
{
    /**
     * Shunt selection logic for low-side current sensing:
     * 
     * For each sector, we have two phases with valid low-side conduction windows.
     * A phase is valid if its low-side MOSFET is ON long enough around the sample point.
     * 
     * For center-aligned PWM (up-down counting):
     * - Low duty cycle means long low-side ON time (most of the period)
     * - High duty cycle means short low-side ON time (brief pulses)
     * 
     * We need: (1 - duty) > min_duty_window
     * Or equivalently: duty < (1 - min_duty_window)
     * 
     * Sector-based selection picks the two phases with lowest duties:
     */

    float min_duty_for_valid = 1.0f - config.min_duty_window;
    
    // For each sector, select the two phases with the longest low-side ON time
    // This corresponds to the two phases with the LOWEST duty cycles
    
    // Sector 1 (0° to 60°):   A max, B mid, C min -> Lowest are B and C
    // Sector 2 (60° to 120°):  B max, A mid, C min -> Lowest are A and C
    // Sector 3 (120° to 180°): B max, C mid, A min -> Lowest are A and C
    // Sector 4 (180° to 240°): C max, B mid, A min -> Lowest are A and B
    // Sector 5 (240° to 300°): C max, A mid, B min -> Lowest are A and B
    // Sector 6 (300° to 360°): A max, C mid, B min -> Lowest are B and C
    
    switch (sector) {
        case SVPWM_SECTOR_1:
            // Max: A, Mid: B, Min: C -> Lowest are B and C
            if (duty_b < min_duty_for_valid && duty_c < min_duty_for_valid) {
                *shunt1 = PHASE_B;
                *shunt2 = PHASE_C;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_2:
            // Max: B, Mid: A, Min: C -> Lowest are A and C
            if (duty_a < min_duty_for_valid && duty_c < min_duty_for_valid) {
                *shunt1 = PHASE_A;
                *shunt2 = PHASE_C;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_3:
            // Max: B, Mid: C, Min: A -> Lowest are A and C
            if (duty_a < min_duty_for_valid && duty_c < min_duty_for_valid) {
                *shunt1 = PHASE_A;
                *shunt2 = PHASE_C;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_4:
            // Max: C, Mid: B, Min: A -> Lowest are A and B
            if (duty_a < min_duty_for_valid && duty_b < min_duty_for_valid) {
                *shunt1 = PHASE_A;
                *shunt2 = PHASE_B;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_5:
            // Max: C, Mid: A, Min: B -> Lowest are A and B
            if (duty_a < min_duty_for_valid && duty_b < min_duty_for_valid) {
                *shunt1 = PHASE_A;
                *shunt2 = PHASE_B;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_6:
            // Max: A, Mid: C, Min: B -> Lowest are B and C
            if (duty_b < min_duty_for_valid && duty_c < min_duty_for_valid) {
                *shunt1 = PHASE_B;
                *shunt2 = PHASE_C;
                return 1;
            }
            break;
    }
    
    // Fallback: If sector-based selection fails (e.g., near overmodulation),
    // select the two phases with absolute lowest duty cycles
    if (duty_a <= duty_b && duty_a <= duty_c) {
        // A is lowest
        *shunt1 = PHASE_A;
        *shunt2 = (duty_b < duty_c) ? PHASE_B : PHASE_C;
    } else if (duty_b <= duty_a && duty_b <= duty_c) {
        // B is lowest
        *shunt1 = PHASE_B;
        *shunt2 = (duty_a < duty_c) ? PHASE_A : PHASE_C;
    } else {
        // C is lowest
        *shunt1 = PHASE_C;
        *shunt2 = (duty_a < duty_b) ? PHASE_A : PHASE_B;
    }
    
    // Check if fallback selection is valid
    float duty_sh1 = ((*shunt1 == PHASE_A) ? duty_a : ((*shunt1 == PHASE_B) ? duty_b : duty_c));
    float duty_sh2 = ((*shunt2 == PHASE_A) ? duty_a : ((*shunt2 == PHASE_B) ? duty_b : duty_c));
    
    if (duty_sh1 < min_duty_for_valid && duty_sh2 < min_duty_for_valid) {
        return 1;  // Valid
    }
    
    return 0;  // No valid shunts (extreme case, shouldn't happen normally)
}

float SVPWM_CalculateTriggerPoint(SVPWM_Sector_t sector, float duty_a, float duty_b, float duty_c,
                                   Phase_t shunt1, Phase_t shunt2)
{
    /**
     * In Center-Aligned Mode:
     * - Timer counts UP from 0 to PERIOD, then DOWN from PERIOD to 0.
     * - High-side is ON when CNT < CCR (duty * PERIOD).
     * - Low-side  is ON when CNT > CCR.
     *
     * ADC trigger (TIM1_CH4 falling edge) fires when CNT = CCR4 during up-count.
     * Both selected shunts' low-sides are conducting when CNT > max(CCR_sh1, CCR_sh2).
     *
     * So the trigger must be placed ABOVE the higher of the two shunt duties,
     * plus a settling offset to let the current stabilize after switching.
     *
     * trigger_point = max(duty_sh1, duty_sh2) + trigger_offset
     */

    float duty_sh1 = (shunt1 == PHASE_A) ? duty_a : ((shunt1 == PHASE_B) ? duty_b : duty_c);
    float duty_sh2 = (shunt2 == PHASE_A) ? duty_a : ((shunt2 == PHASE_B) ? duty_b : duty_c);

    float max_shunt_duty = (duty_sh1 > duty_sh2) ? duty_sh1 : duty_sh2;
    float trigger_point = max_shunt_duty + config.trigger_offset;

    if (trigger_point > 0.998f) trigger_point = 0.998f;
    if (trigger_point < 0.05f)  trigger_point = 0.05f;

    return trigger_point;
}
