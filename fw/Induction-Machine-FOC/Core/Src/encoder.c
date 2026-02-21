#include "encoder.h"
#include <math.h>

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
  enc->index_found = 0U;
  enc->index_offset_counts = 0;
  enc->position_zeroed = 0;
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

void Encoder_Update(Encoder_Handle_t *enc, float dt_sec)
{
  if ((enc == NULL) || (enc->htim == NULL) || (enc->counts_per_rev == 0U)) {
    return;
  }

  if (dt_sec <= 0.0f) {
    return;
  }

  uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
  uint32_t max_count = (uint32_t)__HAL_TIM_GET_AUTORELOAD(enc->htim) + 1U;
  int32_t delta = 0;
  uint8_t index_event = (__HAL_TIM_GET_FLAG(enc->htim, TIM_FLAG_IDX) != RESET) ? 1U : 0U;

  if (index_event != 0U) {
    __HAL_TIM_CLEAR_FLAG(enc->htim, TIM_FLAG_IDX);
  }

  if ((index_event != 0U) && (enc->index_found == 0U)) {
    // TIM index mode resets CNT asynchronously; rebase software state at first index.
    int32_t counts_from_index = (int32_t)count;
    if (counts_from_index > (int32_t)(max_count / 2U)) {
      counts_from_index -= (int32_t)max_count;
    }

    enc->position_counts = counts_from_index;
    enc->index_offset_counts = 0;
    enc->position_zeroed = enc->position_counts;
    enc->index_found = 1U;
    enc->last_count = count;
    delta = 0;
  } else {
    delta = (int32_t)count - (int32_t)enc->last_count;

    if (delta > (int32_t)(max_count / 2)) {
      delta -= (int32_t)max_count;
    } else if (delta < -((int32_t)max_count / 2)) {
      delta += (int32_t)max_count;
    }

    enc->position_counts += delta;
    enc->last_count = count;

    if (index_event != 0U) {
      // If first-index mode is disabled, keep zero lock deterministic.
      enc->index_offset_counts = enc->position_counts;
      enc->index_found = 1U;
    }

    enc->position_zeroed = enc->position_counts - enc->index_offset_counts;
  }

  float counts_per_sec = (float)delta / dt_sec;
  float rev_per_sec = counts_per_sec / (float)enc->counts_per_rev;
  enc->speed_rad_per_sec = fabs(rev_per_sec) * ENCODER_TWO_PI;
  enc->speed_rpm = fabs(rev_per_sec) * 60.0f;

  const float MAX_PHYSICAL_RPM = 6000.0f;
  
  if (enc->speed_rpm > -MAX_PHYSICAL_RPM && enc->speed_rpm < MAX_PHYSICAL_RPM) {
      // Valid sample
      enc->speed_rpm_filtered = 0.9f * enc->speed_rpm_filtered + 0.1f * enc->speed_rpm;
  } else {
      // Invalid sample (spike), ignore.
  }
  
  // Use filtered value for control/display
  enc->speed_rad_per_sec = (enc->speed_rpm_filtered / 60.0f) * ENCODER_TWO_PI;
}

float Encoder_GetMechanicalAngleRad(const Encoder_Handle_t *enc)
{
  if ((enc == NULL) || (enc->counts_per_rev == 0U)) {
    return 0.0f;
  }

  uint32_t mod = Encoder_Modulo(enc->position_zeroed, enc->counts_per_rev);
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

  return enc->position_zeroed;
}

uint8_t Encoder_HasIndex(const Encoder_Handle_t *enc)
{
  if (enc == NULL) {
    return 0U;
  }

  return enc->index_found;
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
  enc->position_zeroed = enc->position_counts - enc->index_offset_counts;
  enc->last_count = (uint16_t)counter;
}
