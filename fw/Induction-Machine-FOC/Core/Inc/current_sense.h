#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "stm32g4xx_hal.h"
#include "svpwm.h"

// ==============================================================================
// HARDWARE PARAMETERS 
// ==============================================================================
// Shunt resistor value in Ohms
#define LISTEN_R_SHUNT         0.06f 
// Operational Amplifier Gain (V/V)
#define LISTEN_OPAMP_GAIN      2.1   
// ADC Reference Voltage (V)
#define ADC_VREF               3.3f
// ADC Resolution (2^12 - 1)
#define ADC_MAX_COUNT          4095.0f
// Voltage Offset (V) - for bidirectional sensing
#define ADC_OFFSET_V           (ADC_VREF / 2.0f)
// Current polarity: set to 1.0f or -1.0f to match shunt wiring
#define CURRENT_SENSE_SIGN     -1.0f

// DC Bus Sensing
// Coefficient 1/125 means V_adc = V_bus / 125. So Factor = 125.
#define DC_BUS_CONVERSION_FACTOR 125.0f

// Calibration constants
#define CURRENT_OFFSET_SAMPLES 64U  // Number of samples for offset calibration

// Current offset structure for 3-shunt topology
typedef struct {
    float offset_a;  // Phase A offset (Volts)
    float offset_b;  // Phase B offset (Volts)
    float offset_c;  // Phase C offset (Volts)
    uint8_t calibrated; // Calibration done flag
} CurrentOffset_t;

typedef struct {
    float Ia;   // Phase A current (Amps)
    float Ib;   // Phase B current (Amps)
    float Ic;   // Phase C current (Amps)
    float Vbus; // DC Bus Voltage (Volts)
} CurSense_Data_t;

/**
 * @brief Initialize current sensing module
 */
void CurrentSense_Init(void);

/**
 * @brief Calibrate current sensor offsets (call before motor starts)
 * Averages multiple samples with motor off to determine ADC zero-current offset
 */
void CurrentSense_Calibrate(void);

/**
 * @brief Get current calibration status
 * @return 1 if calibrated, 0 if not
 */
uint8_t CurrentSense_IsCalibrated(void);

/**
 * @brief Start the ADC Injected Conversions (Enable Hardware Trigger)
 */
void CurrentSense_Start(void);

/**
 * @brief  Read the latest injected ADC values (3-shunt, PWM-triggered)
 *         Reads directly from JDR registers.
 * @return CurSense_Data_t Structure containing Ia, Ib, Ic, Vbus
 */
CurSense_Data_t CurrentSense_Read(void);

/**
 * @brief  Read currents using dynamic shunt selection
 *         Measures two valid phases and reconstructs the third using Kirchhoff's current law (Ia + Ib + Ic = 0)
 * @param  shunt1: First phase to measure
 * @param  shunt2: Second phase to measure
 * @return CurSense_Data_t Structure containing Ia, Ib, Ic, Vbus
 */
CurSense_Data_t CurrentSense_ReadWithShuntSelection(Phase_t shunt1, Phase_t shunt2);

/**
 * @brief  Get raw ADC value for a specific phase
 * @param  phase: Phase to read (PHASE_A, PHASE_B, or PHASE_C)
 * @return Raw ADC value (12-bit)
 */
uint32_t CurrentSense_GetRawADC(Phase_t phase);


#endif /* CURRENT_SENSE_H */
