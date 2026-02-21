#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <math.h>

typedef struct {
    float alpha;
    float beta;
} Clarke_Out_t;

typedef struct {
    float d;
    float q;
} Park_Out_t;

/**
 * @brief Clarke Transformation
 * Converts 3-phase currents (Ia, Ib) to 2-phase stationary frame (Alpha, Beta).
 * Assumes balanced system where Ia + Ib + Ic = 0, so Ic is not needed.
 * @param Ia Phase A current
 * @param Ib Phase B current
 * @param Ic Phase C current
 * @return Clarke_Out_t Structure containing alpha and beta components
 */
Clarke_Out_t Clarke_Transform(float Ia, float Ib, float Ic);

/**
 * @brief Park Transformation
 * Converts stationary frame (Alpha, Beta) to rotating frame (d, q).
 * @param alpha Stationary frame alpha component
 * @param beta Stationary frame beta component
 * @param sin_theta Sine of the rotor angle
 * @param cos_theta Cosine of the rotor angle
 * @return Park_Out_t Structure containing d and q components
 */
Park_Out_t Park_Transform(float alpha, float beta, float sin_theta, float cos_theta);

/**
 * @brief Inverse Park Transformation
 * Converts rotating frame (d, q) back to stationary frame (Alpha, Beta).
 * @param d Rotating frame d component
 * @param q Rotating frame q component
 * @param sin_theta Sine of the rotor angle
 * @param cos_theta Cosine of the rotor angle
 * @return Clarke_Out_t Structure containing alpha and beta components
 */
Clarke_Out_t Inv_Park_Transform(float d, float q, float sin_theta, float cos_theta);

#endif /* FOC_MATH_H */
