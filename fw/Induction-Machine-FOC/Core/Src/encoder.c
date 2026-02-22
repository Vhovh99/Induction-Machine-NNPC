#include "encoder.h"
#include <math.h>
#include <stdbool.h>

#define ENCODER_QE_MULT 4U
#define ENCODER_TWO_PI 6.28318530718f

static uint32_t Encoder_Modulo(int32_t value, uint32_t mod)
{
    int32_t result = value % (int32_t)mod;
    if (result < 0) {
      result += (int32_t)mod;
    }
    return (uint32_t)result;
}

void Encoder_Init(Encoder_Handle_t *enc, TIM_HandleTypeDef *htim, uint32_t ppr)
{
    if ((enc == NULL) || (htim == NULL) || (ppr == 0U)) {
      return;
    }

    enc->htim = htim;
    enc->ppr = ppr;
    enc->counts_per_rev = ppr * ENCODER_QE_MULT;
    enc->last_count = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    enc->position_counts = 0;
    enc->speed_rad_per_sec = 0.0f;
    enc->speed_rpm = 0.0f;
    enc->speed_rpm_filtered = 0.0f;
    enc->delta_counts_last = 0;
    enc->position_counts_old = 0;
    enc->time_accumulated_sec = 0.0f;
    enc->speed_sample_count = 0;
    enc->index_found = 0;
}

void Encoder_Start(Encoder_Handle_t *enc)
{
    if ((enc == NULL) || (enc->htim == NULL)) {
      return;
    }

    HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
    __HAL_TIM_CLEAR_FLAG(enc->htim, TIM_FLAG_IDX);
    __HAL_TIM_CLEAR_FLAG(enc->htim, TIM_FLAG_IERR);
    enc->last_count = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
}

static inline float lpf1(float y, float x, float alpha)
{
    // y = alpha*y + (1-alpha)*x
    return alpha * y + (1.0f - alpha) * x;
}

void Encoder_Update(Encoder_Handle_t *enc, float dt_sec)
{
    if (!enc || !enc->htim || enc->counts_per_rev == 0U || dt_sec <= 0.0f) {
        return;
    }

    TIM_HandleTypeDef *htim = enc->htim;

    // Read HW counter early
    uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    uint32_t mod = (uint32_t)__HAL_TIM_GET_AUTORELOAD(htim) + 1U; // CNT modulus

    // Detect index (Z)
    uint8_t index_event = (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_IDX) != RESET) ? 1U : 0U;
    if (index_event) {
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_IDX);

        // Treat index as absolute mechanical zero.
        // Rebase software position to 0 *now*.
        enc->position_counts = 0;

        // Rebase last_count to current counter value so next delta is correct.
        enc->last_count = cnt;

        enc->position_counts_old = enc->position_counts;
        enc->time_accumulated_sec = 0.0f;
        enc->speed_sample_count = 0;
        enc->index_found = 1;
        return;
    } else {
        // Normal incremental update            
        int32_t delta = (int32_t)cnt - (int32_t)enc->last_count;
        int32_t half = (int32_t)(mod / 2U);

        if (delta > half)       delta -= (int32_t)mod;
        else if (delta < -half) delta += (int32_t)mod;

        // Apply configured direction sign so mechanical angle increases in the desired direction.
        delta *= ENCODER_DIR_SIGN;

        enc->position_counts += delta;
        enc->last_count = cnt;
        enc->delta_counts_last = delta;
    }


    
    // Accumulate time and samples
    enc->time_accumulated_sec += dt_sec;
    enc->speed_sample_count++;
    
    // Calculate speed every N samples or when enough time has elapsed
    const uint8_t SPEED_SAMPLES = 10;  // Measure over 10 samples to reduce quantization
    const float MIN_TIME_WINDOW = 0.0005f;  // 0.5ms minimum
    
    if (enc->speed_sample_count >= SPEED_SAMPLES || enc->time_accumulated_sec >= MIN_TIME_WINDOW) {
        int32_t total_delta = enc->position_counts - enc->position_counts_old;
        float counts_per_sec = (float)total_delta / enc->time_accumulated_sec;
        float rev_per_sec = counts_per_sec / (float)enc->counts_per_rev;
        
        enc->speed_rpm = rev_per_sec * 60.0f;
        
        // Spike reject using physical max
        const float MAX_PHYSICAL_RPM = 6000.0f;
        if (enc->speed_rpm > -MAX_PHYSICAL_RPM && enc->speed_rpm < MAX_PHYSICAL_RPM) {
            // 1st order LPF
            enc->speed_rpm_filtered = lpf1(enc->speed_rpm_filtered, enc->speed_rpm, 0.85f);
        }
        
        enc->speed_rad_per_sec = (enc->speed_rpm_filtered / 60.0f) * ENCODER_TWO_PI;
        
        // Reset accumulator
        enc->position_counts_old = enc->position_counts;
        enc->time_accumulated_sec = 0.0f;
        enc->speed_sample_count = 0;
    }
}

float Encoder_GetMechanicalAngleRad(const Encoder_Handle_t *enc)
{
    if ((enc == NULL) || (enc->counts_per_rev == 0U)) {
      return 0.0f;
    }

    uint32_t mod = Encoder_Modulo(enc->position_counts, enc->counts_per_rev);
    return ((float)mod / (float)enc->counts_per_rev) * ENCODER_TWO_PI;
}

float Encoder_GetElectricalAngleRad(const Encoder_Handle_t *enc, uint8_t pole_pairs)
{
    if ((enc == NULL) || (pole_pairs == 0U)) {
      return 0.0f;
    }

    float angle = Encoder_GetMechanicalAngleRad(enc) * (float)pole_pairs;
    while (angle >= ENCODER_TWO_PI) {
      angle -= ENCODER_TWO_PI;
    }
    while (angle < 0.0f) {
      angle += ENCODER_TWO_PI;
    }
    return angle;
}

float Encoder_GetSpeedRadPerSec(const Encoder_Handle_t *enc)
{
    if (enc == NULL) {
      return 0.0f;
    }

    return enc->speed_rad_per_sec;
}

float Encoder_GetSpeedRpm(const Encoder_Handle_t *enc)
{
    if (enc == NULL) {
      return 0.0f;
    }

    return enc->speed_rpm_filtered;
}

int32_t Encoder_GetPositionCounts(const Encoder_Handle_t *enc)
{
    if (enc == NULL) {
      return 0;
    }

    return enc->position_counts;
}

void Encoder_ResetPosition(Encoder_Handle_t *enc, int32_t counts)
{
    if ((enc == NULL) || (enc->htim == NULL)) {
      return;
    }

    uint32_t max_count = (uint32_t)__HAL_TIM_GET_AUTORELOAD(enc->htim) + 1U;
    uint32_t counter = Encoder_Modulo(counts, max_count);

    __HAL_TIM_SET_COUNTER(enc->htim, counter);
    enc->position_counts = counts;
    enc->last_count = (uint16_t)counter;
}
