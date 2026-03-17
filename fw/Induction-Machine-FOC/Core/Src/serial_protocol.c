#include "serial_protocol.h"
#include "foc_control.h"
#include "relay_control.h"
#include <string.h>
#include "main.h"

/* ---- External references set by main.c ---- */
extern Motor_Control_t motor_control;
extern uint8_t run_foc_loop;

/* Runtime-modifiable references (written by protocol, read by ISR) */
volatile float proto_speed_ref  = 0.0f;
volatile float proto_id_ref     = 1.13f; /* Nominal magnetising current (A) */

/* ---- CRC-8/MAXIM (polynomial 0x31, init 0x00) ---- */
static uint8_t crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ---- Send a complete frame ---- */
static void send_frame(Proto_Handle_t *proto, uint8_t cmd, const void *payload, uint8_t len)
{
    /* Skip if previous DMA TX still in progress */
    if (proto->huart->gState != HAL_UART_STATE_READY)
        return;

    uint8_t *buf = proto->tx_buf;
    buf[0] = PROTO_SYNC_BYTE;
    buf[1] = cmd;
    buf[2] = len;
    if (len > 0 && payload != NULL) {
        memcpy(&buf[3], payload, len);
    }
    buf[3 + len] = crc8(&buf[1], 2 + len); /* CRC over CMD + LEN + PAYLOAD */
    HAL_UART_Transmit_DMA(proto->huart, buf, 4 + len);
}

/* ---- Send ACK ---- */
static void send_ack(Proto_Handle_t *proto, uint8_t cmd_echo)
{
    send_frame(proto, RSP_ACK, &cmd_echo, 1);
}

/* ---- Send NACK ---- */
static void send_nack(Proto_Handle_t *proto, uint8_t cmd_echo, uint8_t error_code)
{
    uint8_t payload[2] = { cmd_echo, error_code };
    send_frame(proto, RSP_NACK, payload, 2);
}

/* ---- Process a complete received frame ---- */
static void process_frame(Proto_Handle_t *proto)
{
    uint8_t cmd = proto->rx_cmd;
    uint8_t len = proto->rx_len;
    uint8_t *p  = proto->rx_payload;

    switch (cmd) {
    case CMD_MOTOR_START:
        if (len != 0) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        run_foc_loop = 1;
        motor_control.state = MOTOR_STATE_IDLE; /* triggers flux build → running */
        PWM_START();
        send_ack(proto, cmd);
        break;

    case CMD_MOTOR_STOP:
        if (len != 0) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        run_foc_loop = 0;
        proto_speed_ref = 0.0f;
        /* Zero PWM outputs */
        PWM_STOP();
        motor_control.state = MOTOR_STATE_IDLE; 
        send_ack(proto, cmd);
        break;

    case CMD_SET_SPEED_REF:
        if (len != 4) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        {
            float rpm;
            memcpy(&rpm, p, 4);
            proto_speed_ref = rpm * (3.14159265f / 30.0f); /* RPM -> rad/s */
        }
        send_ack(proto, cmd);
        break;

    case CMD_SET_ID_REF:
        if (len != 4) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        {
            float val;
            memcpy(&val, p, 4);
            proto_id_ref = val;
        }
        send_ack(proto, cmd);
        break;

    case CMD_SET_CURRENT_PI:
        if (len != 8) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        {
            float kp, ki;
            memcpy(&kp, &p[0], 4);
            memcpy(&ki, &p[4], 4);
            motor_control.id_controller.Kp = kp;
            motor_control.id_controller.Ki = ki;
            motor_control.iq_controller.Kp = kp;
            motor_control.iq_controller.Ki = ki;
        }
        send_ack(proto, cmd);
        break;

    case CMD_SET_SPEED_PI:
        if (len != 8) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        {
            float kp, ki;
            memcpy(&kp, &p[0], 4);
            memcpy(&ki, &p[4], 4);
            motor_control.speed_controller.Kp = kp;
            motor_control.speed_controller.Ki = ki;
        }
        send_ack(proto, cmd);
        break;

    case CMD_GET_STATUS:
        if (len != 0) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        {
            Status_Packet_t st;
            st.motor_state = (uint8_t)motor_control.state;
            st.omega_ref   = proto_speed_ref * (30.0f / 3.14159265f); /* rad/s -> RPM */
            st.id_ref      = proto_id_ref;
            st.fault_flags = 0;
            st.uptime_s    = (uint16_t)(HAL_GetTick() / 1000);
            Proto_SendStatus(proto, &st);
        }
        break;

    case CMD_SET_TELEMETRY_DIV:
        if (len != 2) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        {
            uint16_t div;
            memcpy(&div, p, 2);
            proto->telem_divider = div;
        }
        send_ack(proto, cmd);
        break;

    case CMD_SET_LOAD:
        if (len != 1) { send_nack(proto, cmd, ERR_BAD_LENGTH); break; }
        Relay_SetLoad(p[0]);
        send_ack(proto, cmd);
        break;

    default:
        send_nack(proto, cmd, ERR_UNKNOWN_CMD);
        break;
    }
}

/* ---- Public API ---- */

void Proto_Init(Proto_Handle_t *proto, UART_HandleTypeDef *huart)
{
    memset(proto, 0, sizeof(*proto));
    proto->huart = huart;
    proto->rx_state = PROTO_STATE_SYNC;
    proto->telem_divider = 10; /* default: telemetry every 10 ms (100 Hz) */
}

void Proto_StartReceive(Proto_Handle_t *proto)
{
    HAL_UART_Receive_DMA(proto->huart, proto->dma_rx_buf, PROTO_DMA_RX_BUF_SIZE);
}

/* ---- Feed a single byte into the RX state machine ---- */
static void process_byte(Proto_Handle_t *proto, uint8_t byte)
{
    switch (proto->rx_state) {
    case PROTO_STATE_SYNC:
        if (byte == PROTO_SYNC_BYTE)
            proto->rx_state = PROTO_STATE_CMD;
        break;

    case PROTO_STATE_CMD:
        proto->rx_cmd = byte;
        proto->rx_state = PROTO_STATE_LEN;
        break;

    case PROTO_STATE_LEN:
        if (byte > PROTO_MAX_PAYLOAD) {
            proto->rx_state = PROTO_STATE_SYNC;
        } else {
            proto->rx_len = byte;
            proto->rx_idx = 0;
            proto->rx_state = (byte == 0) ? PROTO_STATE_CRC : PROTO_STATE_PAYLOAD;
        }
        break;

    case PROTO_STATE_PAYLOAD:
        proto->rx_payload[proto->rx_idx++] = byte;
        if (proto->rx_idx >= proto->rx_len)
            proto->rx_state = PROTO_STATE_CRC;
        break;

    case PROTO_STATE_CRC:
        {
            uint8_t tmp[2 + PROTO_MAX_PAYLOAD];
            tmp[0] = proto->rx_cmd;
            tmp[1] = proto->rx_len;
            if (proto->rx_len > 0)
                memcpy(&tmp[2], proto->rx_payload, proto->rx_len);
            uint8_t expected = crc8(tmp, 2 + proto->rx_len);

            if (byte == expected) {
                process_frame(proto);
            }
        }
        proto->rx_state = PROTO_STATE_SYNC;
        break;
    }
}

void Proto_Poll(Proto_Handle_t *proto)
{
    /* DMA write head = buffer size - remaining count */
    uint16_t dma_wr_idx = PROTO_DMA_RX_BUF_SIZE
                        - __HAL_DMA_GET_COUNTER(proto->huart->hdmarx);

    while (proto->dma_rd_idx != dma_wr_idx) {
        uint8_t byte = proto->dma_rx_buf[proto->dma_rd_idx];
        proto->dma_rd_idx = (proto->dma_rd_idx + 1) % PROTO_DMA_RX_BUF_SIZE;
        process_byte(proto, byte);
    }
}

void Proto_SendTelemetry(Proto_Handle_t *proto, const Telemetry_Packet_t *telem)
{
    send_frame(proto, RSP_TELEMETRY, telem, sizeof(Telemetry_Packet_t));
}

void Proto_SendStatus(Proto_Handle_t *proto, const Status_Packet_t *status)
{
    send_frame(proto, RSP_STATUS, status, sizeof(Status_Packet_t));
}
