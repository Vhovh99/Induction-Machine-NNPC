#include "foc_math.h"

Clarke_Out_t Clarke_Transform(float Ia, float Ib, float Ic) {
    Clarke_Out_t out;
    // I_alpha = Ia - (1/2) * Ib - (1/2)*Ic
    // I_beta = (√3/2)*Ib - (√3/2)*Ic
    out.alpha = Ia - 0.5f * Ib - 0.5f * Ic;
    out.beta = 0.866025403784f * (Ib - Ic);  // √3/2 ≈ 0.866025403784
    return out;
}

Park_Out_t Park_Transform(float alpha, float beta, float sin_theta, float cos_theta) {
    Park_Out_t out;
    // Id = alpha * cos(θ) + beta * sin(θ)
    // Iq = -alpha * sin(θ) + beta * cos(θ)
    out.d = alpha * cos_theta + beta * sin_theta;
    out.q = -alpha * sin_theta + beta * cos_theta;
    return out;
}

Clarke_Out_t Inv_Park_Transform(float d, float q, float sin_theta, float cos_theta) {
    Clarke_Out_t out;
    // Alpha = Id * cos(θ) - Iq * sin(θ)
    // Beta  = Id * sin(θ) + Iq * cos(θ)
    out.alpha = d * cos_theta - q * sin_theta;
    out.beta = d * sin_theta + q * cos_theta;
    return out;
}
