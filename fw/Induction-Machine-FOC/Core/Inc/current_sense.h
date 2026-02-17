#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "stm32g4xx_hal.h"

// ==============================================================================
// HARDWARE PARAMETERS 
// ==============================================================================
// Shunt resistor value in Ohms
#define LISTEN_R_SHUNT         0.06f 
// Operational Amplifier Gain (V/V)
#define LISTEN_OPAMP_GAIN      2.1f   
// ADC Reference Voltage (V)
#define ADC_VREF               3.3f
// ADC Resolution (2^12 - 1)
#define ADC_MAX_COUNT          4095.0f
// Voltage Offset (V) - for bidirectional sensing
#define ADC_OFFSET_V           (ADC_VREF / 2.0f)

// DC Bus Sensing
// Coefficient 1/125 means V_adc = V_bus / 125. So Factor = 125.
#define DC_BUS_CONVERSION_FACTOR 125.0f

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
 * @brief Start the ADC Injected Conversions (Enable Hardware Trigger)
 */
void CurrentSense_Start(void);

/**
 * @brief  Read the latest Injected ADC values (triggered by PWM)
 *         Reads directly from JDR registers.
 * @return CurSense_Data_t Structure containing Ia, Ib, Ic, Vbus
 */
CurSense_Data_t CurrentSense_Read(void);

/**
 * @brief  Perform zero-current calibration
 *         Call this when NO current is flowing through the motor phases
 *         to measure and store the ADC offset values.
 * @param  samples: Number of samples to average (default: 100)
 */
void CurrentSense_Calibrate(uint16_t samples);

#endif /* CURRENT_SENSE_H */
