#ifndef __SERIAL_PROTOCOL_H
#define __SERIAL_PROTOCOL_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* -------- Frame Format -------- 
 * | SYNC (1) | CMD (1) | LEN (1) | PAYLOAD (0..56) | CRC8 (1) |
 * SYNC = 0xAA
 * CMD  = command/response ID
 * LEN  = payload length in bytes (0..56)
 * CRC8 = CRC-8/MAXIM over CMD + LEN + PAYLOAD
 */

#define PROTO_SYNC_BYTE         0xAA
#define PROTO_MAX_PAYLOAD       56
#define PROTO_FRAME_OVERHEAD    3   /* SYNC + CMD + LEN (CRC is after payload) */
#define PROTO_MAX_FRAME_SIZE    (PROTO_FRAME_OVERHEAD + PROTO_MAX_PAYLOAD + 1)
#define PROTO_DMA_RX_BUF_SIZE   128  /* DMA circular receive buffer size */

/* -------- Command IDs (Host → MCU) -------- */
#define CMD_MOTOR_START         0x01
#define CMD_MOTOR_STOP          0x02
#define CMD_SET_SPEED_REF       0x03  /* payload: float omega_ref (RPM) */
#define CMD_SET_ID_REF          0x04  /* payload: float id_ref (A) */
#define CMD_SET_CURRENT_PI      0x05  /* payload: float Kp, float Ki */
#define CMD_SET_SPEED_PI        0x06  /* payload: float Kp, float Ki */
#define CMD_GET_STATUS          0x07  /* no payload */
#define CMD_SET_TELEMETRY_DIV   0x08  /* payload: uint16_t divider (0 = off) */

/* -------- Response IDs (MCU → Host) -------- */
#define RSP_ACK                 0x80  /* payload: uint8_t cmd_echo */
#define RSP_NACK                0x81  /* payload: uint8_t cmd_echo, uint8_t error_code */
#define RSP_TELEMETRY           0x82  /* payload: Telemetry_Packet_t */
#define RSP_STATUS              0x83  /* payload: Status_Packet_t */

/* -------- Error Codes -------- */
#define ERR_UNKNOWN_CMD         0x01
#define ERR_BAD_LENGTH          0x02
#define ERR_BAD_CRC             0x03
#define ERR_INVALID_VALUE       0x04

/* -------- Telemetry Packet (36 bytes) -------- */
typedef struct {
    float id;             // d-axis current (A)
    float iq;             // q-axis current (A)
    float vbus;           // DC bus voltage (V)
    float omega_m;        // Mechanical speed (RPM)
    float ia;             // Phase A current (A)
    float ib;             // Phase B current (A)
    float ic;             // Phase C current (A)
    float theta_e;        // Electrical angle — integrated omega_e (rad)
    float torque_e;       // Estimated electromagnetic torque (N·m)
} __attribute__((packed)) Telemetry_Packet_t;

/* -------- Status Packet (16 bytes) -------- */
typedef struct {
    uint8_t  motor_state;     // Motor_state_e
    float    omega_ref;       // Current speed reference (RPM)
    float    id_ref;          // Current id reference (A)
    uint8_t  fault_flags;     // Fault bits
    uint16_t uptime_s;        // Uptime in seconds
} __attribute__((packed)) Status_Packet_t;

/* -------- Protocol State Machine -------- */
typedef enum {
    PROTO_STATE_SYNC,
    PROTO_STATE_CMD,
    PROTO_STATE_LEN,
    PROTO_STATE_PAYLOAD,
    PROTO_STATE_CRC,
} Proto_State_t;

typedef struct {
    UART_HandleTypeDef *huart;

    /* DMA circular receive buffer */
    uint8_t dma_rx_buf[PROTO_DMA_RX_BUF_SIZE];
    uint16_t dma_rd_idx;    /* read index into dma_rx_buf */

    /* RX state machine */
    Proto_State_t rx_state;
    uint8_t rx_cmd;
    uint8_t rx_len;
    uint8_t rx_idx;
    uint8_t rx_payload[PROTO_MAX_PAYLOAD];

    /* Telemetry */
    uint16_t telem_divider;  /* telemetry interval in ms (0 = off) */
    uint32_t telem_last_ms;  /* HAL_GetTick() of last telemetry send */

    /* TX buffer */
    uint8_t tx_buf[PROTO_MAX_FRAME_SIZE];
} Proto_Handle_t;

/* -------- API -------- */
void Proto_Init(Proto_Handle_t *proto, UART_HandleTypeDef *huart);
void Proto_StartReceive(Proto_Handle_t *proto);
void Proto_Poll(Proto_Handle_t *proto);
void Proto_SendTelemetry(Proto_Handle_t *proto, const Telemetry_Packet_t *telem);
void Proto_SendStatus(Proto_Handle_t *proto, const Status_Packet_t *status);

#endif /* __SERIAL_PROTOCOL_H */
