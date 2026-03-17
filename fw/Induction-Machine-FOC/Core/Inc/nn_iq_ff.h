#ifndef __NN_IQ_FF_H
#define __NN_IQ_FF_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Neural-network feedforward for iq.
 *
 * Network: 3-input → Dense(16,relu) → Dense(16,relu) → Dense(1,linear)
 * Inputs  (all from Motor_Control_t, same units as telemetry logging):
 *   [0] omega_m      — mechanical speed          (rad/s)
 *   [1] omega_m_ref  — rate-limited speed ref     (rad/s)
 *   [2] dwr_dt       — filtered acceleration      (rad/s²)
 *   [3] imr          — magnetising current        (A)
 * Output:
 *   iq_ff — predicted q-axis torque current  (A)
 *
 * Normalisation constants loaded from iq_ff_meta.json at training time.
 * ----------------------------------------------------------------------- */

void  NN_IqFF_Init(void);
float NN_IqFF_Run(float omega_m, float omega_m_ref, float dwr_dt, float imr);

#endif /* __NN_IQ_FF_H */
