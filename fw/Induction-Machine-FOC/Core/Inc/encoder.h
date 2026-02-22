#ifndef ENCODER_H
#define ENCODER_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Set to -1 to invert encoder direction, +1 for normal.
#define ENCODER_DIR_SIGN (1)

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t ppr;
  uint32_t counts_per_rev;
  uint16_t last_count;
  int32_t position_counts;
  float speed_rad_per_sec;
  float speed_rpm;
  float speed_rpm_filtered;
  int32_t delta_counts_last;
  // Multi-sample speed measurement
  int32_t position_counts_old;
  float time_accumulated_sec;
  uint8_t speed_sample_count;
  bool index_found;
} Encoder_Handle_t;

void Encoder_Init(Encoder_Handle_t *enc, TIM_HandleTypeDef *htim, uint32_t ppr);
void Encoder_Start(Encoder_Handle_t *enc);
void Encoder_Update(Encoder_Handle_t *enc, float dt_sec);
float Encoder_GetMechanicalAngleRad(const Encoder_Handle_t *enc);
float Encoder_GetElectricalAngleRad(const Encoder_Handle_t *enc, uint8_t pole_pairs);
float Encoder_GetSpeedRadPerSec(const Encoder_Handle_t *enc);
float Encoder_GetSpeedRpm(const Encoder_Handle_t *enc);
int32_t Encoder_GetPositionCounts(const Encoder_Handle_t *enc);
void Encoder_ResetPosition(Encoder_Handle_t *enc, int32_t counts);

#endif
