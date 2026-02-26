#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>

#define PWM_PERIOD_TICKS 4250  // Timer counts for 100% duty at 20 kHz with 170 MHz timer clock

/**
 * @brief SVPWM sector enumeration (1-6)
 * Sectors are numbered 1-6 following standard SVPWM convention:
 * Sector 1: 0° to 60°
 * Sector 2: 60° to 120°
 * Sector 3: 120° to 180°
 * Sector 4: 180° to 240°
 * Sector 5: 240° to 300°
 * Sector 6: 300° to 360°
 */
typedef enum {
    SVPWM_SECTOR_1 = 1,
    SVPWM_SECTOR_2 = 2,
    SVPWM_SECTOR_3 = 3,
    SVPWM_SECTOR_4 = 4,
    SVPWM_SECTOR_5 = 5,
    SVPWM_SECTOR_6 = 6
} SVPWM_Sector_t;

/**
 * @brief Phase identification for shunt current sensing
 */
typedef enum {
    PHASE_A = 0,
    PHASE_B = 1,
    PHASE_C = 2
} Phase_t;

/**
 * @brief SVPWM output structure
 * Contains duty cycles, sector information, and shunt selection
 */
typedef struct {
    float duty_a;           // Phase A duty cycle 
    float duty_b;           // Phase B duty cycle
    float duty_c;           // Phase C duty cycle
    SVPWM_Sector_t sector;  // SVPWM sector (1-6)
    Phase_t shunt1;         // First valid shunt for current measurement
    Phase_t shunt2;         // Second valid shunt for current measurement
    uint8_t valid;          // 1 if shunt selection is valid, 0 otherwise
    float trigger_point;    // ADC trigger point (0.0 to 1.0, relative to PWM period)
} SVPWM_Output_t;

/**
 * @brief SVPWM Configuration
 */
typedef struct {
    float min_duty_window;      // Minimum duty cycle for valid shunt (e.g., 0.15 = 15%)
    float trigger_offset;       // Time offset from duty edge to trigger (0.0 to 1.0)
    uint32_t pwm_period_ticks;  // PWM period in timer ticks
} SVPWM_Config_t;

/**
 * @brief Initialize SVPWM module
 * @param config: SVPWM configuration structure
 */
void SVPWM_Init(SVPWM_Config_t *config);

/**
 * @brief Calculate SVPWM duty cycles and determine valid shunts
 * @param v_alpha: Alpha-axis voltage reference (-1.0 to 1.0)
 * @param v_beta: Beta-axis voltage reference (-1.0 to 1.0)
 * @param v_bus: DC Bus voltage (Volts)
 * @return SVPWM_Output_t structure with duty cycles and shunt selection
 */
SVPWM_Output_t SVPWM_Calculate(float v_alpha, float v_beta, float v_bus);

/**
 * @brief Get SVPWM sector from alpha-beta voltages
 * @param v_alpha: Alpha-axis voltage reference
 * @param v_beta: Beta-axis voltage reference
 * @return SVPWM_Sector_t (1-6)
 */
SVPWM_Sector_t SVPWM_GetSector(float v_alpha, float v_beta);

/**
 * @brief Determine which two shunts are valid for measurement
 * This function uses the SVPWM sector and duty cycles to select the best
 * two phases where low-side MOSFETs are ON long enough for accurate sampling.
 * 
 * @param sector: Current SVPWM sector
 * @param duty_a: Phase A duty cycle (0.0 to 1.0)
 * @param duty_b: Phase B duty cycle (0.0 to 1.0)
 * @param duty_c: Phase C duty cycle (0.0 to 1.0)
 * @param shunt1: Pointer to store first valid shunt
 * @param shunt2: Pointer to store second valid shunt
 * @return 1 if valid shunts found, 0 otherwise
 */
uint8_t SVPWM_SelectShunts(SVPWM_Sector_t sector, float duty_a, float duty_b, float duty_c,
                           Phase_t *shunt1, Phase_t *shunt2);

/**
 * @brief Calculate optimal ADC trigger point in PWM period
 * The trigger point should be in the middle of the valid measurement window
 * where both selected low-side MOSFETs are ON.
 * 
 * @param sector: Current SVPWM sector
 * @param duty_a: Phase A duty cycle (0.0 to 1.0)
 * @param duty_b: Phase B duty cycle (0.0 to 1.0)
 * @param duty_c: Phase C duty cycle (0.0 to 1.0)
 * @param shunt1: First valid shunt
 * @param shunt2: Second valid shunt
 * @return Trigger point (0.0 to 1.0, where 0.5 = center of PWM period)
 */
float SVPWM_CalculateTriggerPoint(SVPWM_Sector_t sector, float duty_a, float duty_b, float duty_c,
                                   Phase_t shunt1, Phase_t shunt2);

#endif /* SVPWM_H */
