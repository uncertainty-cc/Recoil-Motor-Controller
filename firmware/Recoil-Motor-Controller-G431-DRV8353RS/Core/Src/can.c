/*
 * can.c
 *
 *  Created on: Sep 9, 2022
 *      Author: TK
 */

#include "can.h"

HAL_StatusTypeDef CAN_init(FDCAN_HandleTypeDef *hfdcan, uint32_t id_filter, uint32_t id_mask) {
  FDCAN_FilterTypeDef filter_config;
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = 0;
  filter_config.FilterType = FDCAN_FILTER_MASK;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = id_filter;
  filter_config.FilterID2 = id_mask;

  HAL_StatusTypeDef status = HAL_OK;

  status |= HAL_FDCAN_ConfigFilter(hfdcan, &filter_config);
  status |= HAL_FDCAN_Start(hfdcan);
  status |= HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  return status;
}

void CAN_getRxFrame(FDCAN_HandleTypeDef *hfdcan, CAN_Frame *rx_frame) {
  FDCAN_RxHeaderTypeDef rx_header;
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_frame->data);

  rx_frame->id = rx_header.Identifier;
  rx_frame->id_type = (rx_header.IdType == FDCAN_STANDARD_ID) ? CAN_ID_STANDARD : CAN_ID_EXTENDED;
  rx_frame->frame_type = (rx_header.RxFrameType == FDCAN_DATA_FRAME) ? CAN_FRAME_DATA : CAN_FRAME_REMOTE;
  switch (rx_header.DataLength) {
    case FDCAN_DLC_BYTES_0:
      rx_frame->size = 0; break;
    case FDCAN_DLC_BYTES_1:
      rx_frame->size = 1; break;
    case FDCAN_DLC_BYTES_2:
      rx_frame->size = 2; break;
    case FDCAN_DLC_BYTES_3:
      rx_frame->size = 3; break;
    case FDCAN_DLC_BYTES_4:
      rx_frame->size = 4; break;
    case FDCAN_DLC_BYTES_5:
      rx_frame->size = 5; break;
    case FDCAN_DLC_BYTES_6:
      rx_frame->size = 6; break;
    case FDCAN_DLC_BYTES_7:
      rx_frame->size = 7; break;
    case FDCAN_DLC_BYTES_8:
      rx_frame->size = 8; break;
    default:
      rx_frame->size = 0;
  }
}

HAL_StatusTypeDef CAN_putTxFrame(FDCAN_HandleTypeDef *hfdcan, CAN_Frame *tx_frame) {
  FDCAN_TxHeaderTypeDef tx_header;

  tx_header.Identifier = tx_frame->id;
  tx_header.IdType = (tx_frame->id_type == CAN_ID_STANDARD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
  tx_header.TxFrameType = (tx_frame->frame_type == CAN_FRAME_DATA) ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME;
  switch (tx_frame->size) {
    case 0:
      tx_header.DataLength = FDCAN_DLC_BYTES_0; break;
    case 1:
      tx_header.DataLength = FDCAN_DLC_BYTES_1; break;
    case 2:
      tx_header.DataLength = FDCAN_DLC_BYTES_2; break;
    case 3:
      tx_header.DataLength = FDCAN_DLC_BYTES_3; break;
    case 4:
      tx_header.DataLength = FDCAN_DLC_BYTES_4; break;
    case 5:
      tx_header.DataLength = FDCAN_DLC_BYTES_5; break;
    case 6:
      tx_header.DataLength = FDCAN_DLC_BYTES_6; break;
    case 7:
      tx_header.DataLength = FDCAN_DLC_BYTES_7; break;
    case 8:
      tx_header.DataLength = FDCAN_DLC_BYTES_8; break;
    default:
      tx_header.DataLength = FDCAN_DLC_BYTES_0;
  }
  tx_header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_frame->data);
}

