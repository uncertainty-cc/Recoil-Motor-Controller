/*
 * rath_can.h
 *
 *  Created on: Jan 31, 2023
 *      Author: TK
 */

#ifndef INC_RATH_CAN_H_
#define INC_RATH_CAN_H_

#include "stm32f4xx_hal.h"
#include "rath_core.h"

#define PYTHONCAN_START_OF_FRAME      0xAA
#define PYTHONCAN_END_OF_FRAME        0xBB

typedef enum {
  CAN_ID_STANDARD,
  CAN_ID_EXTENDED
} CAN_ID_Type;

typedef enum {
  CAN_FRAME_REMOTE,
  CAN_FRAME_DATA
} CAN_Frame_Type;

typedef struct {
  uint32_t id;
  CAN_ID_Type id_type;
  CAN_Frame_Type frame_type;
  uint16_t size;
  uint8_t data[8];
} CAN_Frame;

/**
 * Read a CAN frame from the peripheral RX buffer.
 *
 * @param hcan:     the CAN peripheral handle
 * @param rx_frame: pointer to the CAN_Frame struct to store the received data
 */
void CAN_getRxFrame(CAN_HandleTypeDef *hcan, CAN_Frame *rx_frame);

/**
 * Write a CAN frame to the peripheral TX buffer.
 *
 * @param hcan: the CAN peripheral handle
 * @param tx_frame: pointer to the CAN_Frame struct that contains the data to be transmitted
 */
HAL_StatusTypeDef CAN_putTxFrame(CAN_HandleTypeDef *hcan, CAN_Frame *tx_frame);

#endif /* INC_RATH_CAN_H_ */
