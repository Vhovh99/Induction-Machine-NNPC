#include "vf_control.h"
#include <string.h>

static VF_Control_t vf_ctrl = {0};

/**
 * @brief Initialize V/F control module
 */
void VF_Init(uint16_t pwm_period, float pwm_freq_hz)
{
    memset(&vf_ctrl, 0, sizeof(VF_Control_t));
    
    // PWM parameters
    vf_ctrl.pwm_period = pwm_period;
    vf_ctrl.pwm_freq_hz = pwm_freq_hz;
    
    // Default control parameters
    vf_ctrl.frequency = 10.0f;          // Start with 10 Hz
    vf_ctrl.amplitude = 0.3f;           // 30% modulation index
    vf_ctrl.vf_ratio = 0.02f;           // 0.02 V/Hz default ratio
    
    // Modulation parameters
    vf_ctrl.max_modulation = 1.1547f;   // SVPWM max: 2/sqrt(3)
    vf_ctrl.mod_index_q15 = (int32_t)(0.3f * 32767.0f);
    
    // Operating limits
    vf_ctrl.min_frequency = 0.1f;       // 0.1 Hz minimum
    vf_ctrl.max_frequency = 100.0f;     // 100 Hz maximum
    vf_ctrl.min_amplitude = 0.0f;       // 0% minimum
    vf_ctrl.max_amplitude = 0.95f;      // 95% maximum (leave headroom)
    
    // Phase accumulator
    vf_ctrl.phase_accum = 0;
    VF_SetFrequency(vf_ctrl.frequency); // Calculate initial phase increment
    
    vf_ctrl.enabled = 0;
}

/**
 * @brief Enable V/F control
 */
void VF_Enable(void)
{
    vf_ctrl.enabled = 1;
}

/**
 * @brief Disable V/F control
 */
void VF_Disable(void)
{
    vf_ctrl.enabled = 0;
}

/**
 * @brief Set output frequency
 */
void VF_SetFrequency(float frequency)
{
    // Clamp to limits
    if (frequency < vf_ctrl.min_frequency) {
        frequency = vf_ctrl.min_frequency;
    }
    if (frequency > vf_ctrl.max_frequency) {
        frequency = vf_ctrl.max_frequency;
    }
    
    vf_ctrl.frequency = frequency;
    
    // Calculate phase increment for NCO (Numerically Controlled Oscillator)
    // phase_increment = (f_out / f_pwm) * 2^32
    vf_ctrl.phase_incr = (uint32_t)((double)frequency * 4294967296.0 / (double)vf_ctrl.pwm_freq_hz);
}

/**
 * @brief Set output amplitude (modulation index)
 */
void VF_SetAmplitude(float amplitude)
{
    // Clamp to limits
    if (amplitude < vf_ctrl.min_amplitude) {
        amplitude = vf_ctrl.min_amplitude;
    }
    if (amplitude > vf_ctrl.max_amplitude) {
        amplitude = vf_ctrl.max_amplitude;
    }
    if (amplitude > vf_ctrl.max_modulation) {
        amplitude = vf_ctrl.max_modulation;
    }
    
    vf_ctrl.amplitude = amplitude;
    
    // Convert to Q15 format for efficient fixed-point processing
    vf_ctrl.mod_index_q15 = (int32_t)(amplitude * 32767.0f);
}

/**
 * @brief Set V/F ratio for constant flux operation
 */
void VF_SetVFRatio(float vf_ratio)
{
    vf_ctrl.vf_ratio = vf_ratio;
}

/**
 * @brief Apply V/F ratio automatically
 */
void VF_ApplyVFRatio(float frequency)
{
    if (vf_ctrl.vf_ratio > 0.0f) {
        // Calculate amplitude from V/F ratio
        float amplitude = frequency * vf_ctrl.vf_ratio;
        
        // Apply minimum voltage to overcome friction at low speeds
        if (amplitude < 0.15f && frequency > 1.0f) {
            amplitude = 0.15f;
        }
        
        VF_SetAmplitude(amplitude);
    }
}

/**
 * @brief Get current frequency
 */
float VF_GetFrequency(void)
{
    return vf_ctrl.frequency;
}

/**
 * @brief Get current amplitude
 */
float VF_GetAmplitude(void)
{
    return vf_ctrl.amplitude;
}

/**
 * @brief Get V/F control state
 */
VF_Control_t* VF_GetState(void)
{
    return &vf_ctrl;
}

/**
 * @brief Get phase increment for DMA-based waveform generation
 */
uint32_t VF_GetPhaseIncrement(void)
{
    return vf_ctrl.phase_incr;
}

/**
 * @brief Get modulation index in Q15 format
 */
int32_t VF_GetModulationIndexQ15(void)
{
    return vf_ctrl.mod_index_q15;
}

/**
 * @brief Update phase accumulator (called at PWM rate if needed)
 */
void VF_UpdatePhaseAccumulator(void)
{
    vf_ctrl.phase_accum += vf_ctrl.phase_incr;
}

/**
 * @brief Reset phase accumulator to zero
 */
void VF_ResetPhase(void)
{
    vf_ctrl.phase_accum = 0;
}
