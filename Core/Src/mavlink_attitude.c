/**
  ******************************************************************************
  * @file    mavlink_attitude.c
  * @brief   Independent MAVLink ATTITUDE (MSG ID 30) packaging and UART TX library.
  ******************************************************************************
  */

#include "mavlink_attitude.h"
#include "common/mavlink.h"

_Static_assert(MAVLINK_ATTITUDE_BUF_LEN >= MAVLINK_MAX_PACKET_LEN,
               "MAVLINK_ATTITUDE_BUF_LEN must be >= MAVLINK_MAX_PACKET_LEN");

uint16_t MAVLink_Attitude_Pack(uint8_t system_id,
                               uint8_t component_id,
                               const MAVLink_Attitude_t *att,
                               uint8_t *tx_buf,
                               uint16_t buf_size)
{
    if (att == NULL || tx_buf == NULL || buf_size < MAVLINK_MAX_PACKET_LEN) {
        return 0;
    }

    static mavlink_message_t msg;

    mavlink_msg_attitude_pack(system_id,
                              component_id,
                              &msg,
                              att->time_boot_ms,
                              att->roll,
                              att->pitch,
                              att->yaw,
                              att->rollspeed,
                              att->pitchspeed,
                              att->yawspeed);

    return mavlink_msg_to_send_buffer(tx_buf, &msg);
}

HAL_StatusTypeDef MAVLink_Attitude_SendEx(UART_HandleTypeDef *huart,
                                          uint8_t system_id,
                                          uint8_t component_id,
                                          const MAVLink_Attitude_t *att)
{
    if (huart == NULL || att == NULL) {
        return HAL_ERROR;
    }

    static uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = MAVLink_Attitude_Pack(system_id, component_id, att, tx_buf, sizeof(tx_buf));
    if (len == 0) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(huart, tx_buf, len, MAVLINK_ATTITUDE_UART_TIMEOUT);
}

HAL_StatusTypeDef MAVLink_Attitude_Send(UART_HandleTypeDef *huart,
                                        const MAVLink_Attitude_t *att)
{
    return MAVLink_Attitude_SendEx(huart,
                                   MAVLINK_ATTITUDE_DEFAULT_SYS_ID,
                                   MAVLINK_ATTITUDE_DEFAULT_COMP_ID,
                                   att);
}
