/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_hal_legacy.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_hrtim.h"
#include <complex.h>
#include <stdint.h>
#include "sine_op.h"
#include "foc_math.h"
#include "foc_control.h"
#include "vf_control.h"
#include "current_sense.h"
#include "encoder.h"
#include "stm32g4xx_hal_tim.h"
#include "relay_control.h"
#include "svpwm.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief Motor Control Parameter Structure
 */
typedef struct {
  float frequency;      // Output frequency in Hz
  float amplitude;      // Output amplitude (0.0 to 1.0)
  float phase;          // Current phase angle in radians
  float phase_step;     // Phase increment per PWM cycle
  uint16_t period;      // PWM period in timer ticks
} Motor_Control_t;

/**
 * @brief Control mode enumeration
 */
typedef enum {
  CONTROL_VF = 0,      // Voltage/Frequency control (open-loop)
  CONTROL_FOC = 1      // Field Oriented Control (closed-loop current)
} Control_Mode_t;

typedef enum {
  FOC_BRINGUP_IDLE = 0,
  FOC_BRINGUP_PREPARE,
  FOC_BRINGUP_RAMP_ID,
  FOC_BRINGUP_HOLD_ID,
  FOC_BRINGUP_RAMP_IQ,
  FOC_BRINGUP_HOLD_RUN,
  FOC_BRINGUP_RAMP_DOWN
} FOC_Bringup_State_t;

typedef enum {
  FOC_AUTOCAL_IDLE = 0,
  FOC_AUTOCAL_PREPARE_POINT,
  FOC_AUTOCAL_SETTLE_POINT,
  FOC_AUTOCAL_MEASURE_POINT,
  FOC_AUTOCAL_RAMP_DOWN
} FOC_Autocal_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_BUFFER_SIZE  256
#define BUFF_LEN         DMA_BUFFER_SIZE

#define ADC_RESOOLTUION  4095   // 2^12 - 1 for 12-bit ADC
#define VREF_MV          3300   // millivolts
#define ENCODER_PPR      2000U
#define MOTOR_POLE_PAIRS 2U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CORDIC_HandleTypeDef hcordic;

HRTIM_HandleTypeDef hhrtim1;
DMA_HandleTypeDef hdma_hrtim1_a;
DMA_HandleTypeDef hdma_hrtim1_c;
DMA_HandleTypeDef hdma_hrtim1_d;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* Temperature Variable */
float temperature_VTSO = 0.0f;

/* DMA Double Buffers for 3 Phases */
static uint32_t dma_buffer_phase_a[DMA_BUFFER_SIZE];
static uint32_t dma_buffer_phase_b[DMA_BUFFER_SIZE];
static uint32_t dma_buffer_phase_c[DMA_BUFFER_SIZE];

/* Phase Accumulators */
static uint32_t phase_accumulator = 0;
static uint32_t phase_increment = 0;

/* Control Variables */
static const uint32_t PWM_PERIOD_CYCLES = 4250;
static const float PWM_FREQUENCY_HZ = 20000.0f;
static const float CURRENT_LOOP_TS_SEC = 1.0f / 20000.0f;
static const float DEG_PER_RAD = 57.2957795131f;
// Fixed SVPWM gain: 2/sqrt(3) = 1.1547 in Q15 (37837)
static const int32_t svpwm_gain_q15 = 37837;
// Modulation index in Q15 (0.0 to 1.1547 for SVPWM overmodulation)
static int32_t modulation_index_q15 = (int32_t)(0.30f * 32767.0f);

/* Control Mode Management */
static Control_Mode_t control_mode = CONTROL_VF;

/* FOC voltage references (alpha-beta frame) - updated by FOC_Update() */
static float foc_voltage_alpha = 0.0f;
static float foc_voltage_beta = 0.0f;
static uint32_t foc_isr_budget_cycles = 6000U;
volatile uint32_t foc_isr_cycles_last = 0U;
volatile uint8_t foc_fault_overrun = 0U;
volatile uint8_t foc_fault_adc_sync = 0U;
static volatile uint8_t foc_invalid_sample_count = 0U;

static Encoder_Handle_t encoder;

/* UART Communication Buffer */
#define UART_RX_BUFFER_SIZE 64
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_byte;
static uint16_t uart_rx_index = 0;
static uint8_t uart_new_cmd = 0;

/* Binary Telemetry Packet Structure - Single Sample (20 bytes)
 * Format: Speed, Phase (Electrical angle), Ia, Ib, Ic (currents), Iq (Torque), Timestamp, Flags, CRC8
 */
typedef struct {
    uint8_t header;           // 0xAA sync marker
    uint8_t format_version;   // Version 0x01
    int16_t ia;               // Phase A current (mA scaled)
    int16_t ib;               // Phase B current (mA scaled)
    int16_t ic;               // Phase C current (mA scaled)
    int16_t speed_rpm;        // Motor speed (RPM)
    uint16_t timestamp_ms;    // Timestamp (milliseconds)
    int16_t iq_ma;            // Torque current (mA scaled)
    int16_t theta_elec_deg;   // Electrical angle (degrees, -180 to +180, stored as 0.1 deg resolution)
    uint8_t flags;            // Status flags
    uint8_t checksum;         // CRC8 checksum
} __attribute__((packed)) Telemetry_Binary_t;

/* Telemetry DMA Double Buffer */
#define TELEMETRY_BUFFER_SIZE 128
static uint8_t telemetry_buffer[TELEMETRY_BUFFER_SIZE];

static volatile uint8_t telemetry_enabled = 0;  // Telemetry streaming enable flag
static uint8_t relay_count = 0;  // Current number of active relays (0-12)

/* FOC Variables */
volatile CurSense_Data_t currents;
volatile Clarke_Out_t clarke;
volatile Park_Out_t park;
volatile float theta_rotor = 0.0f; // Rotor angle
volatile float theta_mech = 0.0f; // Rotor angle (mechanical)

/* SVPWM Sampling Variables */
static SVPWM_Output_t svpwm_output = {0};
static uint8_t svpwm_enabled = 1U;  // Enable SVPWM-based shunt selection by default

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

// Telemetry function prototypes
void Telemetry_Start(void);
static uint16_t prepare_telemetry_data(void);

// Motor control function prototypes
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetFrequency(float frequency);
void Motor_SetAmplitude(float amplitude);
void Motor_EnableFOC(void);
void Motor_DisableFOC(void);
void Motor_SetFOC_Id(float Id_ref);
void Motor_SetFOC_Iq(float Iq_ref);
void Motor_Update(void);
void Example_Soft_Start(void);

void Motor_StartOpenLoopFOC(void);
void Motor_TransitionToClosedLoop(void);

void Example_VF_Control_Ramp(void);

void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_DMA_XferHalfCpltCallback(DMA_HandleTypeDef *hdma);

// DMA buffer management
static void fill_block(uint32_t start, uint32_t count);
static void PWM_EnableDmaRequests(void);
static void PWM_DisableDmaRequests(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline int32_t ClampQ15(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32767) {
        return -32767;
    }
    return value;
}

static void PWM_WriteCompareShadow(uint32_t cmp_a, uint32_t cmp_b, uint32_t cmp_c)
{
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, cmp_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, cmp_b);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, cmp_c);
}

static void PWM_SetNeutralDuty(void)
{
    uint32_t neutral_cmp = PWM_PERIOD_CYCLES / 2U;
    PWM_WriteCompareShadow(neutral_cmp, neutral_cmp, neutral_cmp);
}

static void PWM_EnableDmaRequests(void)
{
    // Re-fill the buffer with fresh waveform data
    fill_block(0, BUFF_LEN);
    
    // Register DMA callbacks
    hdma_hrtim1_a.XferCpltCallback = HAL_DMA_XferCpltCallback;
    hdma_hrtim1_a.XferHalfCpltCallback = HAL_DMA_XferHalfCpltCallback;
    
    // Start DMA transfers in circular mode
    HAL_DMA_Start_IT(&hdma_hrtim1_a, (uint32_t)dma_buffer_phase_a, (uint32_t)&HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR, BUFF_LEN);
    HAL_DMA_Start_IT(&hdma_hrtim1_c, (uint32_t)dma_buffer_phase_b, (uint32_t)&HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR, BUFF_LEN);
    HAL_DMA_Start_IT(&hdma_hrtim1_d, (uint32_t)dma_buffer_phase_c, (uint32_t)&HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].CMP1xR, BUFF_LEN);

    __HAL_HRTIM_TIMER_ENABLE_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_DMA_RST);
    __HAL_HRTIM_TIMER_ENABLE_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_DMA_RST);
    __HAL_HRTIM_TIMER_ENABLE_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIM_DMA_RST);
}

static void PWM_DisableDmaRequests(void)
{
    HAL_DMA_Abort(&hdma_hrtim1_a);
    HAL_DMA_Abort(&hdma_hrtim1_c);
    HAL_DMA_Abort(&hdma_hrtim1_d);
    __HAL_DMA_DISABLE_IT(&hdma_hrtim1_a, DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(&hdma_hrtim1_c, DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(&hdma_hrtim1_d, DMA_IT_TC);
    __HAL_HRTIM_TIMER_DISABLE_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_DMA_RST);
    __HAL_HRTIM_TIMER_DISABLE_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_DMA_RST);
    __HAL_HRTIM_TIMER_DISABLE_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIM_DMA_RST);
}

static void PWM_ApplyFocVoltage(float v_alpha, float v_beta)
{
    int16_t va = (int16_t)(v_alpha * 32767.0f);
    int16_t vb = (int16_t)((-0.5f * v_alpha + 0.866025f * v_beta) * 32767.0f);
    int16_t vc = (int16_t)((-0.5f * v_alpha - 0.866025f * v_beta) * 32767.0f);

    int32_t va_sv = ((int32_t)va * svpwm_gain_q15) >> 15;
    int32_t vb_sv = ((int32_t)vb * svpwm_gain_q15) >> 15;
    int32_t vc_sv = ((int32_t)vc * svpwm_gain_q15) >> 15;

    uint32_t cmp_a = sine_to_cmp((int16_t)ClampQ15(va_sv), PWM_PERIOD_CYCLES, Q15_MAX);
    uint32_t cmp_b = sine_to_cmp((int16_t)ClampQ15(vb_sv), PWM_PERIOD_CYCLES, Q15_MAX);
    uint32_t cmp_c = sine_to_cmp((int16_t)ClampQ15(vc_sv), PWM_PERIOD_CYCLES, Q15_MAX);

    PWM_WriteCompareShadow(cmp_a, cmp_b, cmp_c);
}

static void FOC_EnterSafeState(void)
{
    FOC_Disable();
    control_mode = CONTROL_VF;
    foc_voltage_alpha = 0.0f;
    foc_voltage_beta = 0.0f;
    PWM_SetNeutralDuty();
}

static void FOC_InitCycleCounter(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (uart_rx_byte == '\n' || uart_rx_byte == '\r') {
             uart_rx_buffer[uart_rx_index] = '\0'; // Null terminate
             uart_rx_index = 0;
             uart_new_cmd = 1; // Signal main loop
        } else {
             if (uart_rx_index < UART_RX_BUFFER_SIZE - 1) {
                 uart_rx_buffer[uart_rx_index++] = uart_rx_byte;
             } else {
                 // Buffer overflow, reset
                 uart_rx_index = 0;
             }
        }
        // Restart reception
        HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
    }
}

void Process_Command(uint8_t *cmd_buffer)
{
    char *cmd = (char*)cmd_buffer;
    float val_f;
    uint8_t has_arg;
    
    // Command format: "CHAR VALUE"
    // V/F Control commands:
    //   F 50.0  -> Set Frequency 50Hz
    //   A 0.5   -> Set Amplitude 0.5
    //   R 1000  -> Set RPM (example)
    //   S       -> Motor Start
    //   X       -> Motor Stop
    //
    // FOC Control commands:
    //   M V     -> Switch to V/F control Mode
    //   M F     -> Switch to FOC control Mode (encoder required)
    //   O       -> Start Open-Loop FOC (flux align + ramp)
    //   T       -> Transition to closed-loop FOC (after open-loop)
    //   I 0.5   -> Set Id (flux) reference current (Amperes)
    //   Q 1.0   -> Set Iq (torque) reference current (Amperes)
    //   W       -> Query open-loop state, Vbus, frequency, angle, Id, Iq
    //   P       -> Display motor parameters (Rs, Rr, Lm, Ls, Lr)
    //   D       -> Diagnostics: show currents, transforms, angles, errors
    //   N <0|1> -> Disable/enable slip compensation
    //   G <val> -> Set slip gain
    //
    // Relay & Load Control commands:
    //   L <0-12> -> Set number of active load relays (0=none, 12=all)
    //   L+       -> Activate all load relays
    //   L-       -> Deactivate all load relays
    //   L        -> Query current load status
    //
    // Telemetry commands:
    //   Y 1      -> Enable telemetry streaming (1kHz via UART DMA)
    //   Y 0      -> Disable telemetry streaming
    //   Y        -> Query telemetry streaming status
    
    // Parse float manually to avoid sscanf issues on some platforms
    char *ptr = cmd + 1;
    while ((*ptr == ' ') || (*ptr == '\t')) {
        ptr++;
    }
    has_arg = (*ptr != '\0') ? 1U : 0U;
    val_f = has_arg ? strtof(ptr, NULL) : 0.0f;

    switch (cmd[0]) {
        case 'F': // Frequency
        case 'f':
            if (val_f > 0.0f) {
                Motor_SetFrequency(val_f);
            }
            break;
        case 'A': // Amplitude
        case 'a':
            if (val_f >= 0.0f) {
                Motor_SetAmplitude(val_f);
            }
            break;
        case 'R': // RPM
        case 'r':
            if (val_f > 0.0f) {
              // Convert RPM to electrical frequency (Hz)
              // f_elec = RPM * pole_pairs / 60
              float freq_hz = (val_f * MOTOR_POLE_PAIRS) / 60.0f;
              Motor_SetFrequency(freq_hz);
              // Set amplitude proportional to frequency for V/F control
              // V/F keeps voltage/frequency ratio constant for constant flux
              float amplitude = (freq_hz / 50.0f) * 0.5f;  // Scale to nominal 50Hz
              if (amplitude > 0.94f) amplitude = 0.94f;
              Motor_SetAmplitude(amplitude);
            }
            break;
        case 'S': // Start
        case 's':
            Motor_Start();
            break;
        case 'X': // Stop
        case 'x':
            Motor_Stop();
            break;
        case 'M': // Control Mode selection
        case 'm':
            if (cmd[2] == 'V' || cmd[2] == 'v') {
                Motor_DisableFOC();
                HAL_UART_Transmit(&huart3, (uint8_t*)"Mode: V/F\r\n", 11, 500);
            } else if (cmd[2] == 'F' || cmd[2] == 'f') {
              Motor_EnableFOC();
              HAL_UART_Transmit(&huart3, (uint8_t*)"Mode: FOC\r\n", 11, 500);
            }
            break;
        case 'I': // Set Id reference (flux)
        case 'i':
            FOC_SetIdReference(val_f);
            break;
        case 'Q': // Set Iq reference (torque)
        case 'q':
            FOC_SetIqReference(val_f);
            break;
        case 'O': // Open-Loop FOC startup
        case 'o':
            Motor_StartOpenLoopFOC();
            HAL_UART_Transmit(&huart3, (uint8_t*)"Open-Loop FOC: STARTED\r\n", 24, 200);
            break;
        case 'N': // Slip compensation enable
        case 'n':
            if (has_arg != 0U) {
                FOC_Control_t *foc_state = FOC_GetState();
                uint8_t en = (val_f > 0.5f) ? 1U : 0U;
                FOC_SetSlipCompensation(en, foc_state->slip_gain, foc_state->min_id_for_slip);
                HAL_UART_Transmit(&huart3, (uint8_t *)(en ? "Slip: ON\r\n" : "Slip: OFF\r\n"), en ? 10 : 11, 200);
            }
            break;
        case 'G': // Slip gain
        case 'g':
            if (has_arg != 0U) {
                FOC_Control_t *foc_state = FOC_GetState();
                FOC_SetSlipCompensation(foc_state->slip_comp_enabled, val_f, foc_state->min_id_for_slip);
                char msg[64];
                snprintf(msg, sizeof(msg), "Slip gain: %.3f\r\n", FOC_GetState()->slip_gain);
                HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 200);
            }
            break;
        case 'Y': // Start/Stop telemetry streaming
        case 'y':
            if (has_arg != 0U) {
                if (val_f > 0.5f) {
                    telemetry_enabled = 1U;
                    HAL_UART_Transmit(&huart3, (uint8_t*)"Telemetry: ENABLED (1kHz)\r\n", 29, 200);
                    Telemetry_Start();
                } else {
                    telemetry_enabled = 0U;
                    HAL_UART_Transmit(&huart3, (uint8_t*)"Telemetry: DISABLED\r\n", 22, 200);
                }
            } else {
                if (telemetry_enabled != 0U) {
                    HAL_UART_Transmit(&huart3, (uint8_t*)"Telemetry: ENABLED (1kHz)\r\n", 29, 200);
                } else {
                    HAL_UART_Transmit(&huart3, (uint8_t*)"Telemetry: DISABLED\r\n", 22, 200);
                }
            }
            break;
        case 'T': // Transition to Closed-Loop FOC
        case 't':
            {
                FOC_Control_t *foc_before = FOC_GetState();
                float angle_before = foc_before->theta_sync;
                
                Motor_TransitionToClosedLoop();
                
                FOC_Control_t *foc_after = FOC_GetState();
                float angle_after = foc_after->theta_sync;
                
                if (FOC_GetMode() == FOC_MODE_CLOSED_LOOP) {
                    char msg[150];
                    snprintf(msg, sizeof(msg), 
                             "FOC Mode: CLOSED-LOOP\\r\\nAngle sync: %.1f° → %.1f° (Δ=%.1f°)\\r\\n",
                             angle_before * DEG_PER_RAD,
                             angle_after * DEG_PER_RAD,
                             (angle_after - angle_before) * DEG_PER_RAD);
                    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 200);
                } else {
                    HAL_UART_Transmit(&huart3, (uint8_t*)"Transition failed: motor not in COMPLETE state\\r\\n", 49, 200);
                }
            }
            break;
        case 'W': // Query open-loop state and Vbus
        case 'w':
            {
                OpenLoop_State_t ol_state = FOC_GetOpenLoopState();
                FOC_Control_t *foc = FOC_GetState();
                const char* state_names[] = {
                    "IDLE",  "WAIT INDEX",
                    "CALIBRATE", "COMPLETE"
                };
            char msg[300];
            // Calculate slip frequency in Hz for easier interpretation
            float slip_freq_hz = foc->omega_slip / (2.0f * 3.14159265f);
            float slip_angle_deg = foc->theta_slip * DEG_PER_RAD;
            
            // Safely get state name (bounds check)
            const char* state_str = "UNKNOWN";
            if (ol_state < (sizeof(state_names) / sizeof(state_names[0]))) {
                state_str = state_names[ol_state];
            }
            
            snprintf(msg, sizeof(msg),
                 "Vbus:%.1fV OL:%s Freq:%.1fHz Angle:%.1fdeg Id:%.2fA Iq:%.2fA | Idm:%.2f Iqm:%.2f\\r\\n"
                 "Slip: %.2fHz (%.1fdeg) Flux:%.3fWb | ISR:%lu ADC:%u OV:%u SV:%u\\r\\n",
                 foc->Vbus,
                 state_str,
                 foc->openloop_freq,
                 foc->openloop_angle * DEG_PER_RAD,
                 foc->Id_ref,
                 foc->Iq_ref,
                 foc->i_park.d,
                 foc->i_park.q,
                 slip_freq_hz,
                 slip_angle_deg,
                 foc->rotor_flux_wb,
                 (unsigned long)foc_isr_cycles_last,
                 (unsigned int)foc_fault_adc_sync,
                 (unsigned int)foc_fault_overrun,
                 (unsigned int)svpwm_output.valid);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 200);
            }
            break;
        case 'P': // Display motor parameters
        case 'p':
            {
                FOC_Control_t *foc = FOC_GetState();
                char msg[300];
                snprintf(msg, sizeof(msg), 
                         "Motor Parameters:\\r\\n"
                         "  Rs  = %.3f ohm (stator resistance)\\r\\n"
                         "  Rr  = %.3f ohm (rotor resistance)\\r\\n"
                         "  Lm  = %.3f H   (magnetizing inductance)\\r\\n"
                         "  Ls  = %.3f H   (stator inductance)\\r\\n"
                         "  Lr  = %.3f H   (rotor inductance)\\r\\n"
                         "  Vbus= %.1f V   (DC bus voltage)\\r\\n",
                         foc->Rs, foc->Rr, foc->Lm, foc->Ls, foc->Lr, foc->Vbus);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 200);
            }
            break;
        case 'L': // Relay control: L <count> (0-12) | L+ (add all) | L- (remove all) | L (query)
        case 'l':
            if (cmd[1] == '+') {
                // Add all relays
                Relay_SetLoad(12);
                relay_count = 12;
                HAL_UART_Transmit(&huart3, (uint8_t*)"Load: ON (12/12 relays)\r\n", 26, 200);
            } else if (cmd[1] == '-') {
                // Remove all relays
                Relay_SetLoad(0);
                relay_count = 0;
                HAL_UART_Transmit(&huart3, (uint8_t*)"Load: OFF (0/12 relays)\r\n", 26, 200);
            } else if (has_arg != 0U) {
                // Set specific number of relays (0-12)
                uint8_t count = (uint8_t)val_f;
                if (count > 12) count = 12;
                Relay_SetLoad(count);
                relay_count = count;
                char rmsg[64];
                snprintf(rmsg, sizeof(rmsg), "Load: %u/12 relays\r\n", count);
                HAL_UART_Transmit(&huart3, (uint8_t *)rmsg, strlen(rmsg), 200);
            } else {
                // Query current relay state
                char rmsg[64];
                snprintf(rmsg, sizeof(rmsg), "Load: %u/12 relays\r\n", relay_count);
                HAL_UART_Transmit(&huart3, (uint8_t *)rmsg, strlen(rmsg), 200);
            }
            break;
        case 'D': // Diagnostic: Show raw currents, transforms, and angles
        case 'd':
            {
                FOC_Control_t *foc = FOC_GetState();
                char msg[450];
                snprintf(msg, sizeof(msg),
                         "=== FOC DIAGNOSTICS ===\\r\\n"
                         "Currents (ABC): Ia=%.3f Ib=%.3f Ic=%.3f\\r\\n"
                         "Clarke (αβ):    α=%.3f  β=%.3f\\r\\n"
                         "Park (dq):      d=%.3f  q=%.3f\\r\\n"
                         "References:     Id=%.3f Iq=%.3f\\r\\n"
                         "Errors:         Ed=%.3f Eq=%.3f\\r\\n"
                         "Angles: θ_rotor=%.1f° θ_offset=%.1f° θ_slip=%.1f° θ_sync=%.1f°\\r\\n"
                         "Voltages: Vd=%.3f Vq=%.3f Vbus=%.1fV\\r\\n"
                         "Flux: %.4fWb (target: %.4fWb)\\r\\n",
                         currents.Ia, currents.Ib, currents.Ic,
                         foc->i_clarke.alpha, foc->i_clarke.beta,
                         foc->i_park.d, foc->i_park.q,
                         foc->Id_ref, foc->Iq_ref,
                         foc->Id_ref - foc->i_park.d,
                         foc->Iq_ref - foc->i_park.q,
                         theta_rotor * DEG_PER_RAD,
                         foc->theta_offset * DEG_PER_RAD,
                         foc->theta_slip * DEG_PER_RAD,
                         foc->theta_sync * DEG_PER_RAD,
                         foc->Vd, foc->Vq, foc->Vbus,
                         foc->rotor_flux_wb,
                         foc->Lm * foc->Id_ref);
                HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 200);
            }
            break;
        default:
            break;
    }
}

void fill_block(uint32_t start, uint32_t count)
{
    
    // V/F mode: sine wave generation
    for (uint32_t i = 0; i < count; i++) {
        uint32_t phU = phase_accumulator;
        uint32_t phV = phase_accumulator + PHASE_120;
        uint32_t phW = phase_accumulator + PHASE_240;

        int16_t sU = get_sine_q15(phU);
        int16_t sV = get_sine_q15(phV);
        int16_t sW = get_sine_q15(phW);

        // SVPWM Implementation (Min-Max Injection)
        int16_t min_v = sU;
        int16_t max_v = sU;

        if (sV < min_v) min_v = sV;
        if (sV > max_v) max_v = sV;

        if (sW < min_v) min_v = sW;
        if (sW > max_v) max_v = sW;

        // Calculate zero-sequence voltage: -0.5 * (min + max)
        int16_t v_offset = -((int32_t)min_v + max_v) / 2;

        int32_t sU_sv = (int32_t)sU + v_offset;
        int32_t sV_sv = (int32_t)sV + v_offset;
        int32_t sW_sv = (int32_t)sW + v_offset;

        // Apply fixed SVPWM gain (2/sqrt(3))
        sU_sv = (sU_sv * svpwm_gain_q15) >> 15;
        sV_sv = (sV_sv * svpwm_gain_q15) >> 15;
        sW_sv = (sW_sv * svpwm_gain_q15) >> 15;

        // Saturate to ensure int16 valid range (conservative)
        if (sU_sv > 32767) sU_sv = 32767; else if (sU_sv < -32767) sU_sv = -32767;
        if (sV_sv > 32767) sV_sv = 32767; else if (sV_sv < -32767) sV_sv = -32767;
        if (sW_sv > 32767) sW_sv = 32767; else if (sW_sv < -32767) sW_sv = -32767;

        // Apply modulation index only in sine_to_cmp()
        dma_buffer_phase_a[start + i] = sine_to_cmp((int16_t)sU_sv, PWM_PERIOD_CYCLES, modulation_index_q15);
        dma_buffer_phase_b[start + i] = sine_to_cmp((int16_t)sV_sv, PWM_PERIOD_CYCLES, modulation_index_q15);
        dma_buffer_phase_c[start + i] = sine_to_cmp((int16_t)sW_sv, PWM_PERIOD_CYCLES, modulation_index_q15);

        phase_accumulator += phase_increment; // advance one PWM update step
    }

}

/**
 * @brief DMA Half Transfer Complete Callback - refill first half of buffer
 */
void HAL_DMA_XferHalfCpltCallback(DMA_HandleTypeDef *hdma)
{
    // Refill first half of buffer (indices 0 to BUFF_LEN/2-1)
    fill_block(0, BUFF_LEN / 2);
}

/**
 * @brief DMA Full Transfer Complete Callback - refill second half of buffer
 */
void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    // Refill second half of buffer (indices BUFF_LEN/2 to BUFF_LEN-1)
    fill_block(BUFF_LEN / 2, BUFF_LEN / 2);
}

float ADC_Measure_VTSO(void) {
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 200) == HAL_OK) {
        uint16_t adc_value = HAL_ADC_GetValue(&hadc3);
        // Calculate Voltage: V = (ADC / 4095) * 3.3V
        uint16_t voltage_mv = (adc_value * 3300) / 4095;
        
        // Calculate Temperature from Graph (Typ line):
        // y = kx + b; where k = 18.666666667f, b = 700 (mV at 0°C)
        temperature_VTSO = (voltage_mv - 700) / 18.666666667f; // in °C
    }
    HAL_ADC_Stop(&hadc3);
    return temperature_VTSO;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_CORDIC_Init();
  MX_ADC3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
  Encoder_Init(&encoder, &htim2, ENCODER_PPR);
  Encoder_Start(&encoder);
  FOC_InitCycleCounter();
  foc_isr_budget_cycles = (uint32_t)((SystemCoreClock * 0.75f) / (uint32_t)PWM_FREQUENCY_HZ);
  if (foc_isr_budget_cycles == 0U) {
      foc_isr_budget_cycles = 6000U;
  }

  CurrentSense_Init();
  CurrentSense_Start();
  // Start Counters (Master + Slaves)
  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | 
                                         HRTIM_TIMERID_TIMER_A | 
                                         HRTIM_TIMERID_TIMER_C | 
                                         HRTIM_TIMERID_TIMER_D);
  CurrentSense_Calibrate();
    // Start Counters (Master + Slaves)
  HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_MASTER | 
                                         HRTIM_TIMERID_TIMER_A | 
                                         HRTIM_TIMERID_TIMER_C | 
                                         HRTIM_TIMERID_TIMER_D);
  // Initialize FOC control module
  FOC_Init();
  FOC_SetSlipCompensation(1U, 15.0f, 0.2f);

  // Initialize V/F control module
  VF_Init(PWM_PERIOD_CYCLES, PWM_FREQUENCY_HZ);
  
  // Initialize SVPWM module for synchronized current sampling
  SVPWM_Config_t svpwm_config = {
      .min_duty_window = 0.15f,        // 15% minimum low-side ON time for valid measurement
      .trigger_offset = 0.05f,         // 5% offset from duty edge for ADC settling
      .max_modulation_index = 0.8165f, // Standard SVPWM max (sqrt(3)/2)
      .pwm_period_ticks = PWM_PERIOD_CYCLES
  };
  SVPWM_Init(&svpwm_config);

  // Build sine lookup table
  build_sine_lut();
  
  VF_SetFrequency(50.0f); // Start with 50Hz for testing
  phase_increment = VF_GetPhaseIncrement();
  
  // Start DMA-based telemetry transmission
  Telemetry_Start();
  
//   HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (uart_new_cmd) {
        Process_Command(uart_rx_buffer);
        uart_new_cmd = 0;
    }

    if (ADC_Measure_VTSO() > 150.0f) {
        Motor_Stop();
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INJECSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_HRTIM_TRG2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  // Configure CORDIC for sine calculation (optimized for motor control)
  CORDIC_ConfigTypeDef sCordicConfig;
  sCordicConfig.Function = CORDIC_FUNCTION_SINE;      // Sine function
  sCordicConfig.Precision = CORDIC_PRECISION_6CYCLES; // 6 cycles = 24 iterations
  sCordicConfig.Scale = CORDIC_SCALE_0;               // No scaling
  sCordicConfig.NbWrite = CORDIC_NBWRITE_1;           // 1 input (angle only)
  sCordicConfig.NbRead = CORDIC_NBREAD_1;             // 1 output (sine only)
  sCordicConfig.InSize = CORDIC_INSIZE_32BITS;        // 32-bit input
  sCordicConfig.OutSize = CORDIC_OUTSIZE_32BITS;      // 32-bit output
  
  if (HAL_CORDIC_Configure(&hcordic, &sCordicConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_FaultBlankingCfgTypeDef pFaultBlkCfg = {0};
  HRTIM_FaultCfgTypeDef pFaultCfg = {0};
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultPrescalerConfig(&hhrtim1, HRTIM_FAULTPRESCALER_DIV8) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultBlkCfg.Threshold = 8;
  pFaultBlkCfg.ResetMode = HRTIM_FAULTCOUNTERRST_UNCONDITIONAL;
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_FaultCounterConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultBlkCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pFaultCfg.Source = HRTIM_FAULTSOURCE_DIGITALINPUT;
  pFaultCfg.Polarity = HRTIM_FAULTPOLARITY_LOW;
  pFaultCfg.Filter = HRTIM_FAULTFILTER_15;
  pFaultCfg.Lock = HRTIM_FAULTLOCK_READWRITE;
  if (HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_3, &pFaultCfg) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_HRTIM_FaultModeCtl(&hhrtim1, HRTIM_FAULT_3, HRTIM_FAULTMODECTL_ENABLED);
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_MASTER_CMP1;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 8500;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 4250;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 4250;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_BOTH|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_VALLEY) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_UPD;
  pTimerCfg.DMASrcAddress = 0;
  pTimerCfg.DMADstAddress = 0;
  pTimerCfg.DMASize = 1;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_FAULT3;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 2125;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
  pDeadTimeCfg.RisingValue = 200;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 200;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_OUTPUT_TD2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_BOTH|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_VALLEY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_BOTH|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_VALLEY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_EncoderIndexConfigTypeDef sEncoderIndexConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV8;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV8;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sEncoderIndexConfig.Polarity = TIM_ENCODERINDEX_POLARITY_NONINVERTED;
  sEncoderIndexConfig.Prescaler = TIM_ENCODERINDEX_PRESCALER_DIV1;
  sEncoderIndexConfig.Filter = 15;
  sEncoderIndexConfig.FirstIndexEnable = DISABLE;
  sEncoderIndexConfig.Position = TIM_ENCODERINDEX_POSITION_00;
  sEncoderIndexConfig.Direction = TIM_ENCODERINDEX_DIRECTION_UP_DOWN;
  if (HAL_TIMEx_ConfigEncoderIndex(&htim2, &sEncoderIndexConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 170-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RELAY10_Pin|RELAY9_Pin|GPIO_PIN_9|RELAY12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY6_Pin|RELAY1_Pin|RELAY7_Pin|RELAY8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY4_Pin|RELAY5_Pin|RELAY2_Pin|RELAY3_Pin
                          |RELAY11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY10_Pin RELAY9_Pin PC9 RELAY12_Pin */
  GPIO_InitStruct.Pin = RELAY10_Pin|RELAY9_Pin|GPIO_PIN_9|RELAY12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC0 PC1 PC2
                           PC3 PC4 PC5 PC6
                           PC7 PC8 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY6_Pin RELAY1_Pin RELAY7_Pin RELAY8_Pin */
  GPIO_InitStruct.Pin = RELAY6_Pin|RELAY1_Pin|RELAY7_Pin|RELAY8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY4_Pin RELAY5_Pin RELAY2_Pin RELAY3_Pin
                           RELAY11_Pin */
  GPIO_InitStruct.Pin = RELAY4_Pin|RELAY5_Pin|RELAY2_Pin|RELAY3_Pin
                          |RELAY11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
Clarke_Out_t v_alphabeta = {0.0f, 0.0f}; 
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) {
        return;
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    // uint32_t start_cycles = DWT->CYCCNT;

    if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS) == RESET) {
        foc_fault_adc_sync = 1U;
        if (++foc_invalid_sample_count > 1U) {
            FOC_EnterSafeState();
        }
        return;
    }
   
    foc_invalid_sample_count = 0U;
    foc_fault_adc_sync = 0U;

    // Always measure currents for telemetry/monitoring
    CurSense_Data_t current_sample;
    if (svpwm_enabled && svpwm_output.valid) {
        current_sample = CurrentSense_ReadWithShuntSelection(svpwm_output.shunt1, svpwm_output.shunt2);
    } else {
        current_sample = CurrentSense_Read();
    }
    currents = current_sample;
    Encoder_Update(&encoder, CURRENT_LOOP_TS_SEC);
    theta_mech = Encoder_GetMechanicalAngleRad(&encoder);
    theta_rotor = Encoder_GetElectricalAngleRad(&encoder, MOTOR_POLE_PAIRS);
    
    // Check for encoder index during calibration
    if (encoder.index_found && FOC_GetOpenLoopState() == OPENLOOP_WAIT_INDEX) {
        FOC_TriggerCalibration();
        encoder.index_found = 0;  // Clear flag after processing
    }
    
    if (control_mode == CONTROL_FOC) {
        // 1. Update open-loop FOC state machine
        FOC_UpdateOpenLoop(CURRENT_LOOP_TS_SEC);

        // 2. Read encoder position if needed for closed-loop
        FOC_Mode_t foc_mode = FOC_GetMode();

        // 3. Run FOC control
        uint8_t can_run_foc = (foc_mode == FOC_MODE_OPEN_LOOP) ||
                                                    (foc_mode == FOC_MODE_CLOSED_LOOP);

        if (can_run_foc) {
            // Select the correct angle feedback
            float theta_feedback;
            if (foc_mode == FOC_MODE_OPEN_LOOP) {
                // Open-loop: use internally imposed angle
                theta_feedback = FOC_GetState()->openloop_angle;
            } else {
                // Closed-loop: use actual encoder electrical angle
                theta_feedback = Encoder_GetElectricalAngleRad(&encoder, MOTOR_POLE_PAIRS);
            }
            
            v_alphabeta = FOC_Update(current_sample, theta_feedback, CURRENT_LOOP_TS_SEC);
            foc_voltage_alpha = v_alphabeta.alpha;
            foc_voltage_beta = v_alphabeta.beta;

            svpwm_output = SVPWM_Calculate(v_alphabeta.alpha, v_alphabeta.beta);

            PWM_ApplyFocVoltage(v_alphabeta.alpha, v_alphabeta.beta);
        } else {
            foc_voltage_alpha = 0.0f;
            foc_voltage_beta = 0.0f;
            PWM_SetNeutralDuty();
        }
    }

    // foc_isr_cycles_last = DWT->CYCCNT - start_cycles;
    // if (foc_isr_cycles_last > foc_isr_budget_cycles) {
    //     foc_fault_overrun = 1U;
    //     FOC_EnterSafeState();
    // }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    // ========== 1 kHz telemetry and diagnostics ==========
    
    // 1. Update encoder (velocity/speed calculation at 1kHz is sufficient)
    //    Position reads happen at 20kHz in ADC callback only for closed-loop
    static const float TIM6_TS_SEC = 1.0f / 2000.0f;  // 20kHz = 50us
    Encoder_Update(&encoder, TIM6_TS_SEC);
    theta_mech = Encoder_GetMechanicalAngleRad(&encoder);
    
    // 2. Snapshot data from control loop (use values computed by FOC_Update @ 20kHz)
    FOC_Control_t *foc_state = FOC_GetState();
    clarke = foc_state->i_clarke;  // Clarke transform from FOC_Update (no recomputation!)
    park = foc_state->i_park;      // Park transform from FOC_Update (no recomputation!)
    
    // Note: FOC_Update already computes Clarke and Park at 20kHz
    // We just copy the latest values here for telemetry - much faster!
    
    // Telemetry transmission is handled by UART DMA callbacks
  }

}

/**
 * @brief Prepare telemetry buffer with latest data
 * @return Actual length of telemetry string (excluding null terminator)
 */
/**
 * @brief Calculate CRC8 checksum for binary telemetry
 */
static uint8_t Telemetry_CRC8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;  // Polynomial: x^8 + x^5 + x^4 + 1
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Prepare binary telemetry packet (single sample with Phase and Torque)
 * Transmits: Speed, Phase angle, Ia/Ib/Ic currents, Iq (torque), timestamp, status flags
 */
static uint16_t prepare_telemetry_binary(void)
{
    Telemetry_Binary_t packet;
    CurSense_Data_t currents_snapshot;
    float speed_snapshot;
    float theta_snapshot;
    FOC_Control_t *foc_state;
    uint32_t timestamp_ms;

    // Snapshot data safely
    currents_snapshot = currents;
    speed_snapshot = Encoder_GetSpeedRpm(&encoder);
    theta_snapshot = theta_rotor; // Electrical angle in radians
    foc_state = FOC_GetState();
    timestamp_ms = HAL_GetTick();

    // Pack binary structure
    packet.header = 0xAA;                                      // Sync marker
    packet.format_version = 0x01;                              // Version 1
    packet.ia = (int16_t)(currents_snapshot.Ia * 1000.0f);     // Convert A to mA
    packet.ib = (int16_t)(currents_snapshot.Ib * 1000.0f);
    packet.ic = (int16_t)(currents_snapshot.Ic * 1000.0f);
    packet.speed_rpm = (int16_t)(speed_snapshot);              // RPM as signed int16
    
    // Convert electrical angle from radians to degrees (-180 to +180 range)
    float theta_deg = theta_snapshot * DEG_PER_RAD;
    while (theta_deg > 180.0f) theta_deg -= 360.0f;
    while (theta_deg < -180.0f) theta_deg += 360.0f;
    packet.theta_elec_deg = (int16_t)(theta_deg * 10.0f);      // Store as 0.1 degree resolution
    
    // Torque current (Iq) - quadrature component in mA
    packet.iq_ma = (int16_t)(foc_state->Iq_ref * 1000.0f);
    
    packet.timestamp_ms = (uint16_t)(timestamp_ms & 0xFFFF);   // 16-bit rolling timestamp
    packet.flags = control_mode | (foc_fault_overrun << 1) | (foc_fault_adc_sync << 2);
    packet.checksum = Telemetry_CRC8((uint8_t*)&packet, sizeof(packet) - 1);

    // Copy to buffer
    uint16_t packet_len = sizeof(Telemetry_Binary_t);
    if (packet_len <= TELEMETRY_BUFFER_SIZE) {
        memcpy(telemetry_buffer, (uint8_t*)&packet, packet_len);
        return packet_len;
    }
    return 0;
}

/**
 * @brief Prepare telemetry data (binary format only)
 * Contains: Speed, Phase, Ia/Ib/Ic currents, Torque (Iq), Timestamp, Flags, CRC8
 */
static uint16_t prepare_telemetry_data(void)
{
    return prepare_telemetry_binary();
}

/**
 * @brief Start DMA-based telemetry transmission
 */
void Telemetry_Start(void)
{
  if (telemetry_enabled == 0U) {
    return;
  }

  if (huart3.gState != HAL_UART_STATE_READY) {
    return;
  }

  // Prepare first telemetry data
  uint16_t length = prepare_telemetry_data();

  // Start DMA transmission with actual data length
  if (length > 0U) {
    HAL_UART_Transmit_DMA(&huart3, telemetry_buffer, length);
  }
}

/**
 * @brief UART TX Complete Callback - DMA finished sending buffer
 * @note Automatically prepares new data and retriggers transmission when enabled
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
    if (telemetry_enabled != 0U) {
      uint16_t length = prepare_telemetry_data();
      if (length > 0) {
        HAL_UART_Transmit_DMA(&huart3, telemetry_buffer, length);
      }
    }
    }
}

/**
 * @brief Set the fundamental output frequency
 * @param f_out_hz: Desired output frequency in Hz
 * @note Call this to change motor speed. Safe to call during operation.
 */
void set_fundamental_frequency(float f_out_hz)
{
    VF_SetFrequency(f_out_hz);
    phase_increment = VF_GetPhaseIncrement();
}

/**
 * @brief Set the modulation index (amplitude)
 * @param modulation: 0.0 to 1.1547 
 */
void set_modulation_index(float modulation)
{
    VF_SetAmplitude(modulation);
    modulation_index_q15 = VF_GetModulationIndexQ15();
}

/**
 * @brief Start the 3-phase induction motor
 */
void Motor_Start(void)
{
  /* Start all timers and outputs synchronously.
     All slave timers (A, C, D) are configured to reset on Master Period, 
     ensuring Center-Aligned synchronized PWM.
  */
    if (control_mode == CONTROL_FOC) {
        PWM_DisableDmaRequests();
        PWM_SetNeutralDuty();
    } else {
        PWM_EnableDmaRequests();
    }
    // Start Counters (Master + Slaves)
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | 
                                            HRTIM_TIMERID_TIMER_A | 
                                            HRTIM_TIMERID_TIMER_C | 
                                            HRTIM_TIMERID_TIMER_D);

    // Enable Outputs
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
                                            HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 |
                                            HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);


  
}

/**
 * @brief Stop the 3-phase induction motor
 */
void Motor_Stop(void)
{
  // Stop all outputs
  HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1);
  HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2);
  HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TC1);
  HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TC2);
  HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD1);
  HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TD2);
  
  // Stop the timers
  HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_MASTER);
  HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_A);
  HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_C);
  HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_D);
}

/**
 * @brief Set the output frequency in Hz
 * @param frequency: Desired frequency (typically 1-50 Hz for induction motors)
 */
void Motor_SetFrequency(float frequency)
{
  set_fundamental_frequency(frequency);
}

/**
 * @brief Set the modulation amplitude (0.0 to 1.0)
 * @param amplitude: Modulation index (0.0 = off, 1.0 = maximum)
 */
void Motor_SetAmplitude(float amplitude)
{
  set_modulation_index(amplitude);
}

/**
 * @brief Enable FOC control mode
 */
void Motor_EnableFOC(void)
{
    foc_fault_overrun = 0U;
    foc_fault_adc_sync = 0U;
    foc_invalid_sample_count = 0U;
    PWM_DisableDmaRequests();
    PWM_SetNeutralDuty();
    control_mode = CONTROL_FOC;
    FOC_Enable();
}

/**
 * @brief Disable FOC control and return to V/F mode
 */
void Motor_DisableFOC(void)
{
  FOC_Disable();
  control_mode = CONTROL_VF;
  fill_block(0, BUFF_LEN);
  PWM_EnableDmaRequests();
}

/**
 * @brief Set FOC direct axis (flux) current reference
 * @param Id_ref Current in Amperes
 */
void Motor_SetFOC_Id(float Id_ref)
{
  FOC_SetIdReference(Id_ref);
}

/**
 * @brief Set FOC quadrature axis (torque) current reference
 * @param Iq_ref Current in Amperes
 */
void Motor_SetFOC_Iq(float Iq_ref)
{
  FOC_SetIqReference(Iq_ref);
}

/**
 * @brief Start open-loop FOC with flux alignment and frequency ramp
 * 
 * Startup sequence:
 * 1. Flux Alignment (500 ms) - Ramp Id current to magnetize rotor
 * 2. Flux Stabilization (200 ms) - Allow flux to build up
 * 3. Frequency Ramp (3000 ms) - Accelerate to 10 Hz
 * 4. Steady State - Run at constant frequency until encoder locks
 */
void Motor_StartOpenLoopFOC(void)
{
  // Stop any previous control
  Motor_DisableFOC();
  
  // Set control mode to FOC BEFORE calling Motor_Start()
  // This ensures Motor_Start() doesn't enable DMA requests
  control_mode = CONTROL_FOC;
  
  // Ensure PWM is in direct control mode (not DMA)
  PWM_DisableDmaRequests();
  PWM_SetNeutralDuty();

  // Ensure PWM outputs and counters are running (ADC injected triggers depend on HRTIM)
  Motor_Start();

  // Ensure injected ADC conversions are armed (in case they were stopped)
  // CurrentSense_Start(); // do not uncomment this 
  
  // Configure and start open-loop FOC sequence
  // Parameters: align_time_ms, ramp_time_ms, target_freq_hz, flux_id_a
  FOC_StartOpenLoop(
      200,    // 200 ms flux alignment
      6000,   // 6 second frequency ramp
      10.0f,   // Target 10 Hz (300 RPM for 2 pole pair)
      0.4f    // 0.4A magnetizing current
  );
  
  // Configure FOC operating mode and enable
  FOC_SetMode(FOC_MODE_OPEN_LOOP);
  FOC_Enable();
  
  // Start TIM6 for state machine updates (1 kHz)
//   HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief Transition from open-loop to closed-loop FOC
 * Should be called once encoder is locked and motor is rotating
 */
void Motor_TransitionToClosedLoop(void)
{
  if (FOC_GetOpenLoopState() == OPENLOOP_COMPLETE) {
    // CRITICAL: Synchronize slip angle before switching modes
    // This prevents angle discontinuity that would cause motor to stop
    FOC_SynchronizeSlipAngle(theta_rotor);
    
    // Now safe to switch to closed-loop mode (angle is continuous)
    FOC_SetMode(FOC_MODE_CLOSED_LOOP);
    
    // Optionally stop TIM6 updates (open-loop state machine)
    // HAL_TIM_Base_Stop_IT(&htim6);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
