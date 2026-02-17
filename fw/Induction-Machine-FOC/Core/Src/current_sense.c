#include "current_sense.h"
#include "main.h"

/* External handles defined in main.c */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

CurSense_Data_t data = {0};

/* Zero-current calibration offsets (raw ADC counts) */
static uint32_t offset_Ia = 0;
static uint32_t offset_Ib = 0;
static uint32_t offset_Ic = 0;


/**
 * @brief  Convert measured ADC value to Current (Amps)
 * @param  raw_adc: Current ADC reading
 * @param  offset_adc: Calibrated zero-current offset
 */
static float Convers_ADC_To_Current(uint32_t raw_adc, uint32_t offset_adc)
{
    /* Convert raw ADC to voltage */
    float voltage = ((float)raw_adc / ADC_MAX_COUNT) * ADC_VREF;
    float offset_voltage = ((float)offset_adc / ADC_MAX_COUNT) * ADC_VREF;
    
    /* Calculate current: I = (Vout - Voffset) / (G * R_shunt) */
    /* Note: If current flows in reverse, Vout < Voffset, resulting in negative current */
    return (voltage - offset_voltage) / (LISTEN_R_SHUNT * LISTEN_OPAMP_GAIN);
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
    
    /* Set default offsets to mid-scale (will be overwritten by calibration) */
    offset_Ia = (uint32_t)(ADC_MAX_COUNT / 2.0f);
    offset_Ib = (uint32_t)(ADC_MAX_COUNT / 2.0f);
    offset_Ic = (uint32_t)(ADC_MAX_COUNT / 2.0f);
}

void CurrentSense_Start(void)
{
    /* Start Injected Conversions with External Trigger (HRTIM) */
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

}

CurSense_Data_t CurrentSense_Read(void)
{
    // CurSense_Data_t data = {0};

    // --------------------------------------------------------------------------
    // Retrieve Injected Conversion Results (registers updated by Hardware Trigger)
    // --------------------------------------------------------------------------
    // ADC1 JDR1 -> Ia (Injected Rank 1 - Channel 3)
    uint32_t raw_Ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    data.Ia = Convers_ADC_To_Current(raw_Ia, offset_Ia);

    // ADC1 JDR2 -> Ib (Injected Rank 2 - Channel 4)
    uint32_t raw_Ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    data.Ib = Convers_ADC_To_Current(raw_Ib, offset_Ib);

    // ADC2 JDR1 -> Ic (Injected Rank 1 - Channel 3)
    uint32_t raw_Ic = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    data.Ic = Convers_ADC_To_Current(raw_Ic, offset_Ic);

    // ADC2 JDR2 -> Vbus (Injected Rank 2 - Channel 4)
    uint32_t raw_Vbus = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    data.Vbus = Convers_ADC_To_BusVoltage(raw_Vbus);

    return data;
}

void CurrentSense_Calibrate(uint16_t samples)
{
    if (samples == 0) samples = 100;
    
    uint64_t sum_Ia = 0;
    uint64_t sum_Ib = 0;
    uint64_t sum_Ic = 0;
    
    /* Wait a bit for ADC to stabilize */
    HAL_Delay(10);
    
    /* Accumulate multiple samples */
    for (uint16_t i = 0; i < samples; i++) {
        /* Read raw ADC values */
        uint32_t raw_Ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint32_t raw_Ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        uint32_t raw_Ic = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        
        sum_Ia += raw_Ia;
        sum_Ib += raw_Ib;
        sum_Ic += raw_Ic;
        
        HAL_Delay(1); /* Small delay between samples */
    }
    
    /* Calculate average offsets */
    offset_Ia = (uint32_t)(sum_Ia / samples);
    offset_Ib = (uint32_t)(sum_Ib / samples);
    offset_Ic = (uint32_t)(sum_Ic / samples);
}
