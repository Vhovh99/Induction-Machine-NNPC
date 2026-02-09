#ifndef SINE_OP_H
#define SINE_OP_H

#include "stdint.h"

#define LUT_BITS        9
#define LUT_SIZE        (1 << LUT_BITS)  // 512 entries
#define LUT_MASK        (LUT_SIZE - 1)
#define Q15_MAX         32767

#define PHASE_120  ((uint32_t)(0xFFFFFFFFu / 3u))          // ~ 2^32/3
#define PHASE_240  ((uint32_t)(2u * (0xFFFFFFFFu / 3u)))

void build_sine_lut(void);
int16_t get_sine_q15(uint32_t phase);
uint32_t sine_to_cmp(int16_t s_q15, uint32_t per, int32_t m_q15);

#endif // SINE_OP_H