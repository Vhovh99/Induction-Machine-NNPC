#ifndef __DIGITAL_FILTER_H
#define __DIGITAL_FILTER_H

typedef struct {
    float b0, b1, b2;  // Numerator coefficients
    float a1, a2;      // Denominator coefficients
    float x1, x2;      // Previous input samples
    float y1, y2;      // Previous output samples
} SecondOrderLPF_t;

void second_order_lpf_init(SecondOrderLPF_t *filter, float cutoff_freq, float sampling_freq);
float second_order_lpf_process(SecondOrderLPF_t *filter, float input);

#endif /* __DIGITAL_FILTER_H */