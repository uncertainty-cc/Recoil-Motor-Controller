/*
 * rath_can.c
 *
 *  Created on: Jan 31, 2023
 *      Author: TK
 */

#include "rath_can.h"


void CAN_getRxFrame(CAN_HandleTypeDef *hcan, CAN_Frame *rx_frame) {
  CAN_RxHeaderTypeDef rx_header;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_frame->data);

  rx_frame->id = rx_header.StdId;
  rx_frame->id_type = (rx_header.IDE == CAN_ID_STD) ? CAN_ID_STANDARD : CAN_ID_EXTENDED;
  rx_frame->frame_type = (rx_header.RTR == CAN_FRAME_DATA) ? CAN_FRAME_DATA : CAN_FRAME_REMOTE;
  rx_frame->size = rx_header.DLC;
}

HAL_StatusTypeDef CAN_putTxFrame(CAN_HandleTypeDef *hcan, CAN_Frame *tx_frame) {
  CAN_TxHeaderTypeDef tx_header;

  tx_header.StdId = tx_frame->id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = (tx_frame->frame_type == CAN_FRAME_DATA) ? CAN_RTR_DATA : CAN_RTR_REMOTE;
  tx_header.DLC = tx_frame->size;
  tx_header.TransmitGlobalTime = 0U;

  uint32_t tx_mailbox;
  return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_frame->data, &tx_mailbox);
}

