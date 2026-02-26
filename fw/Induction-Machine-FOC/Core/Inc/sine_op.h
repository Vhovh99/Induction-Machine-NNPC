#ifndef SINE_OP_H
#define SINE_OP_H

#include "stdint.h"

#define Q15_MAX         32767


uint32_t sine_to_cmp(int16_t s_q15, uint32_t per, int32_t m_q15);
void pre_calc_sin_cos(float theta, float *sin_theta, float *cos_theta);
float cordic_sin(float angle);
float cordic_cos(float angle);

#endif // SINE_OP_H