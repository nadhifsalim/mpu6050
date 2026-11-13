/**
  ******************************************************************************
  * @file    mavlink_attitude.h
  * @brief   Independent MAVLink ATTITUDE (MSG ID 30) packaging and UART TX library.
  ******************************************************************************
  */

#ifndef MAVLINK_ATTITUDE_H
#define MAVLINK_ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* Default MAVLink identifiers and UART transmit timeout */
#define MAVLINK_ATTITUDE_DEFAULT_SYS_ID     1U
#define MAVLINK_ATTITUDE_DEFAULT_COMP_ID    200U    /* MAV_COMP_ID_IMU */
#define MAVLINK_ATTITUDE_UART_TIMEOUT       HAL_MAX_DELAY

/* Minimum buffer size required to hold any serialized MAVLink packet */
#define MAVLINK_ATTITUDE_BUF_LEN            287U

/* Conversion factor from degrees (deg, deg/s) to radians (rad, rad/s) */
#define MAVLINK_DEG_TO_RAD                  0.01745329251994329577f

/**
  * @brief Attitude payload structure matching MAVLink ATTITUDE (MSG ID 30).
  *        All angles and angular rates must be in radians and rad/s.
  */
typedef struct {
    uint32_t time_boot_ms;  /* Timestamp (time since system boot) [ms] */
    float roll;             /* Roll angle (-pi..+pi) [rad] */
    float pitch;            /* Pitch angle (-pi..+pi) [rad] */
    float yaw;              /* Yaw angle (-pi..+pi) [rad] */
    float rollspeed;        /* Roll angular speed [rad/s] */
    float pitchspeed;       /* Pitch angular speed [rad/s] */
    float yawspeed;         /* Yaw angular speed [rad/s] */
} MAVLink_Attitude_t;

/**
  * @brief  Pack a MAVLink ATTITUDE (MSG ID 30) message and serialize it into a byte buffer.
  * @param  system_id: MAVLink system ID of the sender
  * @param  component_id: MAVLink component ID of the sender
  * @param  att: Pointer to attitude data (angles in rad, rates in rad/s)
  * @param  tx_buf: Output buffer to store the serialized MAVLink frame
  * @param  buf_size: Size of tx_buf in bytes (must be >= MAVLINK_ATTITUDE_BUF_LEN)
  * @retval Number of bytes written to tx_buf, or 0 on invalid arguments
  */
uint16_t MAVLink_Attitude_Pack(uint8_t system_id,
                               uint8_t component_id,
                               const MAVLink_Attitude_t *att,
                               uint8_t *tx_buf,
                               uint16_t buf_size);

/**
  * @brief  Package MAVLink ATTITUDE (MSG ID 30) into a buffer and send it over HAL UART
  *         using explicit system and component IDs.
  * @param  huart: Pointer to initialized STM32 HAL UART handle
  * @param  system_id: MAVLink system ID of the sender
  * @param  component_id: MAVLink component ID of the sender
  * @param  att: Pointer to attitude data (angles in rad, rates in rad/s)
  * @retval HAL status (HAL_OK on success)
  */
HAL_StatusTypeDef MAVLink_Attitude_SendEx(UART_HandleTypeDef *huart,
                                          uint8_t system_id,
                                          uint8_t component_id,
                                          const MAVLink_Attitude_t *att);

/**
  * @brief  Package MAVLink ATTITUDE (MSG ID 30) into a buffer and send it over HAL UART
  *         using default system ID (1) and component ID (200).
  * @param  huart: Pointer to initialized STM32 HAL UART handle
  * @param  att: Pointer to attitude data (angles in rad, rates in rad/s)
  * @retval HAL status (HAL_OK on success)
  */
HAL_StatusTypeDef MAVLink_Attitude_Send(UART_HandleTypeDef *huart,
                                        const MAVLink_Attitude_t *att);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_ATTITUDE_H */
