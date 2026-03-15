#include "nn_iq_ff.h"
#include "network.h"
#include "network_data.h"
#include "network_data_params.h"

/* -----------------------------------------------------------------------
 * Normalisation constants — from iq_ff_meta.json produced at training time.
 * Input order: [omega_m (rad/s), omega_m_ref (rad/s), dwr_dt (rad/s²), imr (A)]
 * ----------------------------------------------------------------------- */
#define NN_X_MEAN_0   ( 74.733f)   // omega_m
#define NN_X_MEAN_1   ( 80.640f)   // omega_m_ref
#define NN_X_MEAN_2   ( -2.446f)   // dwr_dt
#define NN_X_MEAN_3   (  1.087f)   // imr
#define NN_X_STD_0    ( 41.581f)   // omega_m
#define NN_X_STD_1    ( 39.906f)   // omega_m_ref
#define NN_X_STD_2    (506.943f)   // dwr_dt
#define NN_X_STD_3    (  0.217f)   // imr
#define NN_Y_MEAN     ( -0.2178f)
#define NN_Y_STD      (  0.770f)

static ai_handle   nn_handle = AI_HANDLE_NULL;

/* Activation scratch buffer — must be 4-byte aligned, in SRAM */
static AI_ALIGNED(4) ai_u8 nn_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Cached buffer pointers — obtained once at init to avoid per-run overhead */
static ai_buffer *nn_in  = NULL;
static ai_buffer *nn_out = NULL;

/* -----------------------------------------------------------------------
 * NN_IqFF_Init — call once during firmware startup, before the FOC loop.
 * ----------------------------------------------------------------------- */
void NN_IqFF_Init(void)
{
    const ai_handle acts[] = { nn_activations };

    /* weights=NULL → use the flash-resident weights compiled into network_data.c */
    ai_error err = ai_network_create_and_init(&nn_handle, acts, NULL);
    if (err.type != AI_ERROR_NONE) {
        nn_handle = AI_HANDLE_NULL;   /* will cause NN_IqFF_Run to return 0 safely */
        return;
    }

    /* Cache buffer pointers — they remain valid for the lifetime of nn_handle */
    nn_in  = ai_network_inputs_get(nn_handle,  NULL);
    nn_out = ai_network_outputs_get(nn_handle, NULL);
}

/* -----------------------------------------------------------------------
 * NN_IqFF_Run — normalise inputs, run inference, denormalise output.
 * Returns 0.0 if the network was not successfully initialised.
 *
 * Call at ≤ 100 Hz (the dwr_dt LPF cuts off at 10 Hz; faster updates
 * add load without new information).
 * ----------------------------------------------------------------------- */
float NN_IqFF_Run(float omega_m, float omega_m_ref, float dwr_dt, float imr)
{
    if (nn_handle == AI_HANDLE_NULL) {
        return 0.0f;
    }

    float *in_data = (float*)nn_in->data;
    in_data[0] = (omega_m     - NN_X_MEAN_0) / NN_X_STD_0;
    in_data[1] = (omega_m_ref - NN_X_MEAN_1) / NN_X_STD_1;
    in_data[2] = (dwr_dt      - NN_X_MEAN_2) / NN_X_STD_2;
    in_data[3] = (imr         - NN_X_MEAN_3) / NN_X_STD_3;

    ai_network_run(nn_handle, nn_in, nn_out);

    float raw = ((float*)nn_out->data)[0];
    return raw * NN_Y_STD + NN_Y_MEAN;
}
