#include "current_sense.h"
#include "main.h"

/* External handles defined in main.c */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/**
 * @brief  Convert measured ADC value to Current (Amps)
 */
static float Convers_ADC_To_Current(uint32_t raw_adc)
{
    /* Convert raw ADC to voltage */
    float voltage = ((float)raw_adc / ADC_MAX_COUNT) * ADC_VREF;
    
    /* Calculate current: I = (Vout - Vbias) / (G * R_shunt) */
    /* Note: If current flows in reverse, Vout < Vbias, resulting in negative current */
    return (voltage - ADC_OFFSET_V) / (LISTEN_R_SHUNT * LISTEN_OPAMP_GAIN);
}

/**
 * @brief  Convert measured ADC value to DC Bus Voltage (Volts)
 */
static float Convers_ADC_To_BusVoltage(uint32_t raw_adc)
{
    /* Convert raw ADC to voltage */
    float voltage = ((float)raw_adc / ADC_MAX_COUNT) * ADC_VREF;
    
    /* Vbus = V_adc * 125 */
    return voltage * DC_BUS_CONVERSION_FACTOR;
}

void CurrentSense_Init(void)
{
    /* Perform ADC Calibration */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
}

void CurrentSense_Start(void)
{
    /* Start Injected Conversions with External Trigger (HRTIM) */
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }
}

CurSense_Data_t CurrentSense_Read(void)
{
    CurSense_Data_t data = {0};

    // --------------------------------------------------------------------------
    // Retrieve Injected Conversion Results (registers updated by Hardware Trigger)
    // --------------------------------------------------------------------------
    // ADC1 JDR1 -> Ia (Injected Rank 1 - Channel 3)
    uint32_t raw_Ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    data.Ia = Convers_ADC_To_Current(raw_Ia);

    // ADC1 JDR2 -> Ib (Injected Rank 2 - Channel 4)
    uint32_t raw_Ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    data.Ib = Convers_ADC_To_Current(raw_Ib);

    // ADC2 JDR1 -> Ic (Injected Rank 1 - Channel 3)
    uint32_t raw_Ic = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    data.Ic = Convers_ADC_To_Current(raw_Ic);

    // ADC2 JDR2 -> Vbus (Injected Rank 2 - Channel 4)
    uint32_t raw_Vbus = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    data.Vbus = Convers_ADC_To_BusVoltage(raw_Vbus);

    return data;
}
