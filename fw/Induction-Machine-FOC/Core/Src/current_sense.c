#include "current_sense.h"
#include "main.h"

/* External handles defined in main.c */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

CurSense_Data_t data = {0};
static float voltage = 0.0f;
static CurrentOffset_t offsets = {2048, 2048, 2048, 0};  // Default to mid-scale


/**
 * @brief  Convert measured ADC value to Current (Amps)
 * @param  raw_adc: Current ADC reading
 * @param  offset: ADC offset for the current channel
 */
static float Convert_ADC_To_Current(uint32_t raw_adc, float offset)
{
    /* Convert raw ADC to voltage */
    voltage = ((float)raw_adc / ADC_MAX_COUNT) * ADC_VREF;
    
    /* Calculate current: I = (Vout - Voffset) / (G * R_shunt) */
    /* Note: If current flows in reverse, Vout < Voffset, resulting in negative current */
    return CURRENT_SENSE_SIGN * (voltage - offset) / (LISTEN_R_SHUNT * LISTEN_OPAMP_GAIN);
}



/**
 * @brief  Convert measured ADC value to DC Bus Voltage (Volts)
 */
static float Convert_ADC_To_BusVoltage(uint32_t raw_adc)
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

void CurrentSense_Calibrate(void)
{
    float sum_a = 0;
    float sum_b = 0;
    float sum_c = 0;
    
    // Wait for ADC to settle
    HAL_Delay(10);
    
    // Collect samples
    for (uint32_t i = 0; i < CURRENT_OFFSET_SAMPLES; i++) {      
        // Wait for conversion
        HAL_Delay(1);
        
        sum_a += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        sum_b += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        sum_c += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    }
    
    // Calculate averages
    offsets.offset_a = (sum_a / CURRENT_OFFSET_SAMPLES) * (ADC_VREF / ADC_MAX_COUNT);
    offsets.offset_b = (sum_b / CURRENT_OFFSET_SAMPLES) * (ADC_VREF / ADC_MAX_COUNT);
    offsets.offset_c = (sum_c / CURRENT_OFFSET_SAMPLES) * (ADC_VREF / ADC_MAX_COUNT);
    offsets.calibrated = 1;
    
}

uint8_t CurrentSense_IsCalibrated(void)
{
    return offsets.calibrated;
}

void CurrentSense_Start(void)
{
    /* 3-shunt sensing: ADC2 is slave in dual injected simultaneous mode. */
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }

    /* ADC1 (master) starts with IRQ to run the 20 kHz FOC loop on JEOS. */
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

}

CurSense_Data_t CurrentSense_Read(void)
{
    // --------------------------------------------------------------------------
    // Retrieve Injected Conversion Results (registers updated by Hardware Trigger)
    // --------------------------------------------------------------------------
    // ADC1 JDR1 -> Ia (Injected Rank 1 - Channel 3)
    uint32_t raw_Ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

    // ADC1 JDR2 -> Ib (Injected Rank 2 - Channel 4)
    uint32_t raw_Ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    // ADC2 JDR1 -> Ic (Injected Rank 1 - Channel 3)
    uint32_t raw_Ic = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

    // ADC2 JDR2 -> Vbus (Injected Rank 2 - Channel 4)
    uint32_t raw_Vbus = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);

    float ia = Convert_ADC_To_Current(raw_Ia, offsets.offset_a);
    float ib = Convert_ADC_To_Current(raw_Ib, offsets.offset_b);
    float ic = Convert_ADC_To_Current(raw_Ic, offsets.offset_c);
    float i_mean = (ia + ib + ic) * (1.0f / 3.0f);

    // Enforce Ia + Ib + Ic = 0 to reduce residual offset mismatch in 3-shunt sensing.
    data.Ia = ia - i_mean;
    data.Ib = ib - i_mean;
    data.Ic = ic - i_mean;
    data.Vbus = Convert_ADC_To_BusVoltage(raw_Vbus);

    return data;
}

uint32_t CurrentSense_GetRawADC(Phase_t phase)
{
    switch (phase) {
        case PHASE_A:
            return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        case PHASE_B:
            return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        case PHASE_C:
            return HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        default:
            return 0;
    }
}

CurSense_Data_t CurrentSense_ReadWithShuntSelection(Phase_t shunt1, Phase_t shunt2)
{
    /**
     * Read two valid shunts and reconstruct the third using Kirchhoff's current law.
     * 
     * For 3-phase systems: Ia + Ib + Ic = 0
     * Therefore: Ic = -(Ia + Ib), or any permutation
     * 
     * This function measures the two phases with valid low-side conduction windows
     * and calculates the third phase current.
     */
    
    // Read raw ADC values for the two selected shunts
    uint32_t raw_shunt1 = CurrentSense_GetRawADC(shunt1);
    uint32_t raw_shunt2 = CurrentSense_GetRawADC(shunt2);
    
    // Read Vbus (always available on ADC2 JDR2)
    uint32_t raw_Vbus = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    
    // Convert measured shunts to current
    float current_shunt1, current_shunt2;
    
    if (shunt1 == PHASE_A) {
        current_shunt1 = Convert_ADC_To_Current(raw_shunt1, offsets.offset_a);
    } else if (shunt1 == PHASE_B) {
        current_shunt1 = Convert_ADC_To_Current(raw_shunt1, offsets.offset_b);
    } else {
        current_shunt1 = Convert_ADC_To_Current(raw_shunt1, offsets.offset_c);
    }
    
    if (shunt2 == PHASE_A) {
        current_shunt2 = Convert_ADC_To_Current(raw_shunt2, offsets.offset_a);
    } else if (shunt2 == PHASE_B) {
        current_shunt2 = Convert_ADC_To_Current(raw_shunt2, offsets.offset_b);
    } else {
        current_shunt2 = Convert_ADC_To_Current(raw_shunt2, offsets.offset_c);
    }
    
    // Reconstruct all three phase currents
    float ia, ib, ic;
    
    // Determine which phases were measured and reconstruct the third
    uint8_t measured_mask = (1 << shunt1) | (1 << shunt2);
    
    switch (measured_mask) {
        case 0x03:  // PHASE_A (0) and PHASE_B (1) measured
            ia = (shunt1 == PHASE_A) ? current_shunt1 : current_shunt2;
            ib = (shunt1 == PHASE_B) ? current_shunt1 : current_shunt2;
            ic = -(ia + ib);
            break;
            
        case 0x05:  // PHASE_A (0) and PHASE_C (2) measured
            ia = (shunt1 == PHASE_A) ? current_shunt1 : current_shunt2;
            ic = (shunt1 == PHASE_C) ? current_shunt1 : current_shunt2;
            ib = -(ia + ic);
            break;
            
        case 0x06:  // PHASE_B (1) and PHASE_C (2) measured
            ib = (shunt1 == PHASE_B) ? current_shunt1 : current_shunt2;
            ic = (shunt1 == PHASE_C) ? current_shunt1 : current_shunt2;
            ia = -(ib + ic);
            break;
            
        default:
            // Invalid combination (shouldn't happen)
            ia = ib = ic = 0.0f;
            break;
    }
    
    // Store results
    data.Ia = ia;
    data.Ib = ib;
    data.Ic = ic;
    data.Vbus = Convert_ADC_To_BusVoltage(raw_Vbus);
    
    return data;
}

