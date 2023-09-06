/*
 * can.h
 *
 *  Created on: Sep 9, 2022
 *      Author: TK
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"

#define CAN_ID_STANDARD   0U
#define CAN_ID_EXTENDED   1U

#define CAN_FRAME_REMOTE  0U
#define CAN_FRAME_DATA    1U


typedef struct {
  uint32_t id;
  uint8_t id_type;
  uint8_t frame_type;
  uint16_t size;
  uint8_t data[8];
} CAN_Frame;


HAL_StatusTypeDef CAN_init(FDCAN_HandleTypeDef *hfdcan, uint32_t id_filter, uint32_t id_mask);

void CAN_getRxFrame(FDCAN_HandleTypeDef *hfdcan, CAN_Frame *rx_frame);

HAL_StatusTypeDef CAN_putTxFrame(FDCAN_HandleTypeDef *hfdcan, CAN_Frame *tx_frame);


#endif /* INC_CAN_H_ */
