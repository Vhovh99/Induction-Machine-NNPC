#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include <stdint.h>
#include "foc_math.h"
#include "current_sense.h"

/**
 * @brief PID Controller structure
 */
typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float integral;     // Accumulated integral term
    float prev_error;   // Previous error for derivative
    float output_min;   // Output saturation minimum
    float output_max;   // Output saturation maximum
} PID_Controller_t;

/**
 * @brief FOC Operating Mode
 */
typedef enum {
    FOC_MODE_DISABLED = 0,     // FOC disabled
    FOC_MODE_OPEN_LOOP = 1,    // Open-loop FOC (imposed angle)
    FOC_MODE_CLOSED_LOOP = 2   // Closed-loop FOC (encoder feedback)
} FOC_Mode_t;

/**
 * @brief Open-Loop Startup State
 */
typedef enum {
    OPENLOOP_IDLE = 0,         // Not running
    OPENLOOP_FLUX_ALIGN = 1,   // Flux alignment (stationary)
    OPENLOOP_FLUX_RAMP = 2,    // Flux ramp to rated value
    OPENLOOP_ACCEL = 3,        // Frequency ramp up
    OPENLOOP_RUNNING = 4,      // Steady state open-loop
    OPENLOOP_COMPLETE = 5      // Ready for closed-loop transition
} OpenLoop_State_t;

/**
 * @brief FOC Control state structure
 */
typedef struct {
    // Current references (A)
    float Id_ref;       // Direct component reference (flux establishing)
    float Iq_ref;       // Quadrature component reference (torque)
    
    // Voltage outputs (normalized, -1.0 to 1.0)
    float Vd;           // Direct voltage output
    float Vq;           // Quadrature voltage output
    
    // Control mode and enable
    uint8_t enabled;
    FOC_Mode_t mode;    // Operating mode (open-loop vs closed-loop)
    
    // DC Bus voltage for normalization
    float Vbus;         // DC bus voltage (V)
    float Vbus_min;     // Minimum Vbus for operation (V)
    
    // PID controllers for current loops
    PID_Controller_t pid_id;
    PID_Controller_t pid_iq;
    
    // Operating parameters
    float max_voltage;  // Maximum output voltage (for saturation)

    // Induction-machine rotor-flux oriented control (IRFOC)
    uint8_t slip_comp_enabled;  // 1: IRFOC enabled, theta_sync = theta_rotor + theta_slip
    float slip_gain;            // Slip scale factor (1.0 = nominal model)
    float min_id_for_slip;      // Minimum |Id| for slip/flux stability
    float omega_slip;           // Estimated slip electrical speed [rad/s]
    float theta_slip;           // Integrated slip angle [rad]
    float theta_sync;           // Last synchronous angle used by FOC [rad]

    // Motor parameters for rotor flux model
    float Rs;                   // Stator resistance (ohm)
    float Rr;                   // Rotor resistance (ohm)
    float Lm;                   // Magnetizing inductance (H)
    float Ls;                   // Stator inductance (H)
    float Lr;                   // Rotor inductance (H)

    // Rotor flux state (Wb)
    float rotor_flux_wb;        // Estimated rotor flux magnitude (Wb)
    float min_flux_wb;          // Minimum rotor flux for numerical stability

    // Transform results (stored for telemetry)
    Clarke_Out_t i_clarke;      // Clarke transform of measured currents (α-β)
    Park_Out_t i_park;          // Park transform of measured currents (d-q)

    // Open-loop control state
    OpenLoop_State_t openloop_state;
    float openloop_angle;       // Open-loop electrical angle [rad]
    float openloop_freq;        // Open-loop frequency [Hz]
    float openloop_accel;       // Open-loop acceleration [Hz/s]
    uint32_t openloop_time_ms;  // Time in current open-loop state [ms]

} FOC_Control_t;

/**
 * @brief Initialize FOC control module
 * Sets up PID parameters and initial state
 */
void FOC_Init(void);

/**
 * @brief Enable FOC control
 */
void FOC_Enable(void);

/**
 * @brief Disable FOC control (returns to V/F control)
 */
void FOC_Disable(void);

/**
 * @brief Set direct axis current reference (flux)
 * @param Id_ref Desired direct axis current in Amperes
 */
void FOC_SetIdReference(float Id_ref);

/**
 * @brief Set quadrature axis current reference (torque)
 * @param Iq_ref Desired quadrature axis current in Amperes
 */
void FOC_SetIqReference(float Iq_ref);

/**
 * @brief Update FOC control loop
 * Must be called at regular intervals (typically PWM-synchronous, e.g. 20 kHz)
 * @param currents: Phase currents and Vbus from current sensors
 * @param theta: Rotor electrical angle in radians (used in closed-loop mode)
 * @param ts: Sample time in seconds
 * @return Clarke_Out_t Voltage references in alpha-beta frame for PWM modulation
 */
Clarke_Out_t FOC_Update(CurSense_Data_t currents, float theta, float ts);

/**
 * @brief Set FOC operating mode
 * @param mode FOC_MODE_OPEN_LOOP or FOC_MODE_CLOSED_LOOP
 */
void FOC_SetMode(FOC_Mode_t mode);

/**
 * @brief Get current FOC operating mode
 * @return Current FOC mode
 */
FOC_Mode_t FOC_GetMode(void);

/**
 * @brief Start open-loop FOC startup sequence
 * Performs flux alignment, flux ramp, and frequency ramp
 * @param align_time_ms Time for flux alignment (typically 500-1000 ms)
 * @param ramp_time_ms Time for frequency ramp (typically 2000-5000 ms)
 * @param target_freq_hz Target frequency in Hz (typically 5-20 Hz)
 * @param flux_id_a Magnetizing current in Amps (typically 0.3-1.0 A)
 */
void FOC_StartOpenLoop(uint32_t align_time_ms, uint32_t ramp_time_ms, 
                       float target_freq_hz, float flux_id_a);

/**
 * @brief Stop open-loop FOC
 */
void FOC_StopOpenLoop(void);

/**
 * @brief Update open-loop state machine (call synchronized with ADC/FOC at PWM rate)
 * @param dt_sec Time step in seconds since last call
 */
void FOC_UpdateOpenLoop(float dt_sec);

/**
 * @brief Get open-loop state
 * @return Current open-loop state
 */
OpenLoop_State_t FOC_GetOpenLoopState(void);

/**
 * @brief Get FOC control state
 * @return Pointer to FOC control state structure
 */
FOC_Control_t* FOC_GetState(void);

/**
 * @brief Tune PID parameters for Id controller
 * @param Kp, Ki, Kd: PID gains
 */
void FOC_TunePID_Id(float Kp, float Ki, float Kd);

/**
 * @brief Tune PID parameters for Iq controller
 * @param Kp, Ki, Kd: PID gains
 */
void FOC_TunePID_Iq(float Kp, float Ki, float Kd);

/**
 * @brief Configure IRFOC slip compensation (rotor flux oriented)
 * @param enable 1 to enable, 0 to disable
 * @param slip_gain Slip scale factor (1.0 = nominal model)
 * @param min_id Minimum Id magnitude for slip calculation stability
 */
void FOC_SetSlipCompensation(uint8_t enable, float slip_gain, float min_id);

/**
 * @brief Set induction motor electrical parameters used by IRFOC model
 * @param Rs Stator resistance (ohm)
 * @param Rr Rotor resistance (ohm)
 * @param Lm Magnetizing inductance (H)
 * @param Ls Stator inductance (H)
 * @param Lr Rotor inductance (H)
 */
void FOC_SetMotorParams(float Rs, float Rr, float Lm, float Ls, float Lr);

/**
 * @brief Set rotor flux reference in Wb (updates Id reference)
 * @param psi_ref_wb Rotor flux reference in Weber (Wb)
 */
void FOC_SetRotorFluxReference(float psi_ref_wb);

#endif /* FOC_CONTROL_H */
