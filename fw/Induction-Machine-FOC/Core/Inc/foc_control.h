#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "foc_math.h"
#include <stdint.h>
#include "svpwm.h"

/* -------- Tunable FOC Parameters -------- */
#define FOC_PI_MAX_VOLTAGE      220.0f   // PI output clamp (V)
#define FOC_CURRENT_PI_BW       15.0f    // Current loop bandwidth (rad/s)
#define FOC_SPEED_RAMP_RATE     209.0f   // Speed ref slew rate (rad/s per s) ≈ 2000 rpm/s
#define FOC_CURRENT_LPF_ALPHA   0.9969f  // dq current LPF coeff (10 Hz @ 20 kHz)
#define FOC_OMEGA_SL_LPF_ALPHA  0.9906f  // slip freq LPF coeff  (30 Hz @ 20 kHz)
#define FOC_FLUX_BUILD_TIME     0.2f     // Flux build duration (s)
#define FOC_PSI_MIN             1e-3f    // Minimum rotor flux (Wb)
#define FOC_ONE_BY_SQRT3        0.577350269f

typedef struct {
    // <Motor Parameters>
    float Rs;           // Stator resistance
    float Rr;           // Rotor resistance (for induction motors)
    float Lm;           // Magnetizing inductance
    float Ls;           // Stator inductance
    float Lr;           // Rotor inductance (for induction motors)
    float sigma_Ls;     // Stator transient inductance: sigma * Ls
    uint8_t pole_pairs; // Number of pole pairs

    // Derived
    float Tr;           // Rotor time constant (Lr/Rr)

    // Limits / refs
    float id_ref;       // Magnetic flux current reference (A)
    float iq_ref;       // Torque current reference (A)
    float id_max;       // Maximum magnetic flux current (A)
    float iq_max;       // Maximum torque current (A)

    // Sampling
    float Ts;           // Sampling period (s)
} Motor_Parameters_t;


typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_FLUX_BUILD,
    MOTOR_STATE_RUNNING,
} Motor_state_e;

typedef struct {
    float Kp;          // Proportional gain
    float Ki;          // Integral gain
    float integral;    // Integral accumulator
    float out_min;     // Minimum controller output (e.g. -max voltage)
    float out_max;     // Maximum absolute value of controller output
} PI_t;

typedef struct {
    Motor_state_e state;

    // Encoder
    float theta_m;      // Mechanical angle (rad)
    float omega_m;      // Mechanical angular velocity (rad/s)

    // Angle generation
    float theta_sl;       // Slip angle (rad)
    float omega_sl;       // Slip angular velocity (rad/s)
    float omega_sl_filt;  // Low-pass filtered slip frequency (rad/s)
    float theta_e;        // Electrical angle — encoder+slip (rad)
    float theta_e_integ;  // Electrical angle — pure speed integration (rad)

    // Flux estimator ( rotor flux angle estimation i d-axis)
    float psi_r;        // Rotor flux magnitude (Wb)

    // Current feedback in dq
    Park_Out_t i_dq;      // Raw measured d and q axis currents (A)
    Park_Out_t i_dq_filt; // Filtered d and q axis currents (A) — PI feedback

    // Voltage command in dq
    Park_Out_t v_dq;    // d and q axis voltage commands (V)

    // PI controllers
    PI_t id_controller;
    PI_t iq_controller;
    PI_t speed_controller;

    // Speed reference rate limiter
    float omega_ref_ramped;  // Rate-limited speed reference (rad/s)

    // Startup timing
    float flux_build_time; // Time to build up rotor flux (s)

} Motor_Control_t;


void open_loop_voltage_control(float Vd, float Vq, float angle_rad);
void svpwm_test(void);
void Motor_Init(const Motor_Parameters_t *params, Motor_Control_t *ctrl);
void FOC_Control_Loop(Motor_Control_t *ctrl, const Motor_Parameters_t *params,
                      float ia, float ib,
                      float id_ref_cmnd, float omega_ref,
                      float theta_m, float omega_m, float vbus, SVPWM_Output_t *out_svpwm);
void PWM_WriteCompareShadow(float cmp_a, float cmp_b, float cmp_c, float cmp_trigger);
#endif /* __FOC_CONTROL_H */