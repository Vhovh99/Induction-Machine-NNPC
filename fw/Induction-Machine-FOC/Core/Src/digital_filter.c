#include "digital_filter.h"
#include "sine_op.h"

void second_order_lpf_init(SecondOrderLPF_t *lpf, float cutoff_freq_hz, float sampling_freq_hz)
{
    float w0 = 2.0f * 3.14159265359f * cutoff_freq_hz / sampling_freq_hz; // Normalized cutoff frequency (radians/sample)
    float Q = 0.7071f; // Quality factor for Butterworth response 1/sqrt(2)
    float alpha = cordic_sin(w0) / (2.0f * Q);

    float cos_w0 = cordic_cos(w0);
    float a0 = 1.0f + alpha;

    // Butterworth low-pass coefficients
    lpf->b0 = (1.0f - cos_w0) / 2.0f / a0;
    lpf->b1 = (1.0f - cos_w0) / a0;
    lpf->b2 = lpf->b0; // Same as b0
    lpf->a1 = (-2.0f * cos_w0) / a0;
    lpf->a2 = (1.0f - alpha) / a0;

    // Reset state
    lpf->x1 = lpf->x2 = 0.0f;
    lpf->y1 = lpf->y2 = 0.0f;
}

float second_order_lpf_process(SecondOrderLPF_t *lpf, float input)
{
    // Direct Form II Transposed implementation
    float output = lpf->b0 * input + lpf->b1 * lpf->x1 + lpf->b2 * lpf->x2
                   - lpf->a1 * lpf->y1 - lpf->a2 * lpf->y2;

    // Update state
    lpf->x2 = lpf->x1;
    lpf->x1 = input;
    lpf->y2 = lpf->y1;
    lpf->y1 = output;

    return output;
}