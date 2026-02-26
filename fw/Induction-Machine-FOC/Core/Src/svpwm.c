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
        config.min_duty_window = 0.15f;  // 15% minimum duty for valid shunt
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
        // Default to center if no valid shunts (shouldn't happen in normal operation)
        output.trigger_point = 0.5f;
    }
    if (output.trigger_point > 0.95f) output.trigger_point = 0.95f;
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
    
    // Sector 1 (0° to 60°): Phases A and B have lowest duties
    // Sector 2 (60° to 120°): Phases B and C have lowest duties
    // Sector 3 (120° to 180°): Phases A and C have lowest duties (reversed order)
    // Sector 4 (180° to 240°): Phases A and B have lowest duties (reversed order)
    // Sector 5 (240° to 300°): Phases B and C have lowest duties (reversed order)
    // Sector 6 (300° to 360°): Phases A and C have lowest duties
    
    switch (sector) {
        case SVPWM_SECTOR_1:
            // Phases A and B have longest low-side ON
            if (duty_a < min_duty_for_valid && duty_b < min_duty_for_valid) {
                *shunt1 = PHASE_A;
                *shunt2 = PHASE_B;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_2:
            // Phases B and C have longest low-side ON
            if (duty_b < min_duty_for_valid && duty_c < min_duty_for_valid) {
                *shunt1 = PHASE_B;
                *shunt2 = PHASE_C;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_3:
            // Phases C and A have longest low-side ON
            if (duty_c < min_duty_for_valid && duty_a < min_duty_for_valid) {
                *shunt1 = PHASE_C;
                *shunt2 = PHASE_A;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_4:
            // Phases A and B have longest low-side ON (reversed from Sector 1)
            if (duty_a < min_duty_for_valid && duty_b < min_duty_for_valid) {
                *shunt1 = PHASE_A;
                *shunt2 = PHASE_B;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_5:
            // Phases B and C have longest low-side ON (reversed from Sector 2)
            if (duty_b < min_duty_for_valid && duty_c < min_duty_for_valid) {
                *shunt1 = PHASE_B;
                *shunt2 = PHASE_C;
                return 1;
            }
            break;
            
        case SVPWM_SECTOR_6:
            // Phases C and A have longest low-side ON (reversed from Sector 3)
            if (duty_c < min_duty_for_valid && duty_a < min_duty_for_valid) {
                *shunt1 = PHASE_C;
                *shunt2 = PHASE_A;
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
     * For center-aligned PWM (up-down counting):
     * - Timer counts UP from 0 to PERIOD, then DOWN from PERIOD to 0
     * - PWM output is HIGH when timer < CMP (on up-count) and when timer > CMP (on down-count)
     * - Low-side MOSFET is ON when PWM is LOW
     * 
     * For a duty cycle D:
     * - CMP = D * PERIOD
     * - Low-side ON windows: [0, CMP] during up-count and [CMP, PERIOD] during down-count
     * - But with dead-time, effective windows are slightly shorter
     * 
     * The optimal sampling point is at the valley (timer = 0 or PERIOD) where both 
     * selected low-side MOSFETs are guaranteed to be ON.
     * 
     * For STM32 HRTIM center-aligned mode:
     * - The valley (minimum) of the up-down count is the best sampling point
     * - This corresponds to the middle of the PWM period (50%)
     */
    
    // Get duties of selected shunts
    float duty_sh1 = (shunt1 == PHASE_A) ? duty_a : ((shunt1 == PHASE_B) ? duty_b : duty_c);
    float duty_sh2 = (shunt2 == PHASE_A) ? duty_a : ((shunt2 == PHASE_B) ? duty_b : duty_c);
    
    // For center-aligned PWM, the valley (50% point) is ideal
    // The low-side is ON from 0 to (duty * 50%) during valley
    // We want to sample in the middle of the valid window with some offset for settling
    
    // Suppress unused variable warning (keeping for potential future use)
    (void)duty_sh1;
    (void)duty_sh2;
    
    // Calculate trigger point: at valley (0.5) minus a small offset
    // The valley is where the timer transitions from down-count to up-count
    // At this point, low-side MOSFETs with duty < 0.5 are ON
    
    // For typical HRTIM, sampling at the valley (center) is most reliable
    // We can add a small offset if needed for ADC settling time
    float trigger_point = 0.5f - config.trigger_offset;
    
    // Ensure trigger point is within the valid measurement window
    // The window for low-side ON at valley is from 0 to (1 - duty)
    // In center-aligned mode, this translates to trigger before the duty edge
    
    if (trigger_point < 0.1f) {
        trigger_point = 0.1f;  // Minimum 10% into period
    }
    if (trigger_point > 0.9f) {
        trigger_point = 0.9f;  // Maximum 90% into period
    }
    
    return trigger_point;
}
