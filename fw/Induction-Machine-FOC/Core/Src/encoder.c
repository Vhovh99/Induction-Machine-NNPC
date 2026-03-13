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
    enc->position_counts_old = 0;
    enc->index_offset = 0;
    enc->index_first_detected = 0;
    enc->drift_accumulated = 0;
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

        if (!enc->index_first_detected) {
            // First index detection - save current position as reference offset
            enc->index_offset = enc->position_counts;
            enc->index_first_detected = 1;
            enc->drift_accumulated = 0;
        } else {
            // Subsequent index detections - use it to correct accumulated drift
            // Calculate position relative to the first index offset
            int32_t relative_pos = enc->position_counts - enc->index_offset;
            
            // How many complete revolutions should we have completed?
            int32_t expected_revs = relative_pos / (int32_t)enc->counts_per_rev;
            
            // Where should we be? (at offset + expected_revs * counts_per_rev)
            int32_t expected_pos = enc->index_offset + expected_revs * (int32_t)enc->counts_per_rev;
            
            // What's the accumulated drift at index point?
            int32_t drift = enc->position_counts - expected_pos;
            
            // Apply drift correction to remove accumulated error
            enc->position_counts -= drift;
            enc->drift_accumulated = drift;
        }

        // Rebase last_count to current counter value so next delta is correct
        enc->last_count = cnt;
        enc->position_counts_old = enc->position_counts;
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

        // Speed estimation: compute from position delta
        float delta_rad = ((float)delta / (float)enc->counts_per_rev) * ENCODER_TWO_PI;
        float speed_raw = delta_rad / dt_sec;
        // Low-pass filter speed (alpha=0.95 gives ~3Hz cutoff at 20kHz)
        enc->speed_rad_per_sec = lpf1(enc->speed_rad_per_sec, speed_raw, 0.95f);
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
