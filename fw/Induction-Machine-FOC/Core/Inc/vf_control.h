#ifndef VF_CONTROL_H
#define VF_CONTROL_H

#include <stdint.h>

/**
 * @brief V/F Control state structure
 */
typedef struct {
    // Control parameters
    float frequency;        // Output frequency in Hz
    float amplitude;        // Output amplitude (0.0 to 1.0)
    float vf_ratio;         // V/F ratio constant for constant flux operation
    
    // Phase accumulator state
    uint32_t phase_accum;   // Phase accumulator (32-bit for NCO)
    uint32_t phase_incr;    // Phase increment per PWM cycle
    
    // PWM parameters
    uint16_t pwm_period;    // PWM period in timer ticks
    float pwm_freq_hz;      // PWM frequency in Hz
    
    // Modulation
    int32_t mod_index_q15;  // Modulation index in Q15 format
    float max_modulation;   // Maximum modulation index (typically 1.1547 for SVPWM)
    
    // Operating limits
    float min_frequency;    // Minimum frequency (Hz)
    float max_frequency;    // Maximum frequency (Hz)
    float min_amplitude;    // Minimum amplitude
    float max_amplitude;    // Maximum amplitude
    
    // Status
    uint8_t enabled;        // 1 = V/F control active, 0 = disabled
    
} VF_Control_t;

/**
 * @brief Initialize V/F control module
 * @param pwm_period PWM period in timer ticks
 * @param pwm_freq_hz PWM frequency in Hz
 */
void VF_Init(uint16_t pwm_period, float pwm_freq_hz);

/**
 * @brief Enable V/F control
 */
void VF_Enable(void);

/**
 * @brief Disable V/F control
 */
void VF_Disable(void);

/**
 * @brief Set output frequency
 * @param frequency Desired frequency in Hz
 */
void VF_SetFrequency(float frequency);

/**
 * @brief Set output amplitude (modulation index)
 * @param amplitude Desired amplitude (0.0 to 1.0)
 */
void VF_SetAmplitude(float amplitude);

/**
 * @brief Set V/F ratio for constant flux operation
 * @param vf_ratio Voltage/Frequency ratio (V/Hz)
 * @note Set to 0.0 to disable automatic V/F scaling
 */
void VF_SetVFRatio(float vf_ratio);

/**
 * @brief Apply V/F ratio automatically
 * @param frequency Frequency in Hz
 * @note Automatically calculates and sets amplitude based on V/F ratio
 */
void VF_ApplyVFRatio(float frequency);

/**
 * @brief Get current frequency
 * @return Current frequency in Hz
 */
float VF_GetFrequency(void);

/**
 * @brief Get current amplitude
 * @return Current amplitude (0.0 to 1.0)
 */
float VF_GetAmplitude(void);

/**
 * @brief Get V/F control state
 * @return Pointer to V/F control state structure
 */
VF_Control_t* VF_GetState(void);

/**
 * @brief Get phase increment for DMA-based waveform generation
 * @return Phase increment value (32-bit for NCO)
 */
uint32_t VF_GetPhaseIncrement(void);

/**
 * @brief Get modulation index in Q15 format
 * @return Modulation index in Q15 format (0-32767)
 */
int32_t VF_GetModulationIndexQ15(void);

/**
 * @brief Update phase accumulator (called at PWM rate if needed)
 * @note For DMA-based systems, this is usually not needed
 */
void VF_UpdatePhaseAccumulator(void);

/**
 * @brief Reset phase accumulator to zero
 */
void VF_ResetPhase(void);

#endif // VF_CONTROL_H
