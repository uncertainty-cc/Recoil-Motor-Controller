/*
 * app.c
 *
 *  Created on: Jan 24, 2023
 *      Author: TK
 */

#include "app.h"

#define SERIAL_BUFFER_SIZE  5

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;


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


char str[128];


CAN_Frame can_rx_frame;
CAN_Frame can_tx_frame;

uint8_t uart_rx_buffer[64];
uint8_t uart_rx_data_pending = 0U;

uint8_t uart_tx_buffer[64];

uint8_t counter = 0;
uint8_t result;


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  sprintf(str, "canrx\r\n");
  HAL_UART_Transmit_IT(&huart2, (uint8_t *)str, strlen(str));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  HAL_UART_AbortReceive(&huart2);
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  if (!uart_rx_data_pending) {
    uint8_t is_valid_frame = uart_rx_buffer[0] == 0xAAU;
    if (!is_valid_frame) {
      return;
    }
    can_tx_frame.id_type = CAN_ID_STANDARD;
    can_tx_frame.frame_type = CAN_FRAME_DATA;
    uint32_t timestamp = ((uart_rx_buffer[1])
        | (uart_rx_buffer[2] << 8U)
        | (uart_rx_buffer[3] << 16U)
        | (uart_rx_buffer[4] << 24U));
    can_tx_frame.size = uart_rx_buffer[5];
    can_tx_frame.id = ((uart_rx_buffer[6])
        | (uart_rx_buffer[7] << 8U)
        | (uart_rx_buffer[8] << 16U)
        | (uart_rx_buffer[9] << 24U));
    if (can_tx_frame.size) {
      uart_rx_data_pending = 1U;
      HAL_UART_Receive_IT(&huart2, uart_rx_buffer, can_tx_frame.size+1);
      return;
    }
  }
  else {
    memcpy(can_tx_frame.data, uart_rx_buffer, can_tx_frame.size);
  }

  CAN_putTxFrame(&hfdcan1, &can_tx_frame);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

  uart_rx_data_pending = 0U;
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 10);
}

void APP_init() {
  FDCAN_FilterTypeDef filter_config;
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = 0;
  filter_config.FilterType = FDCAN_FILTER_MASK;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = 1;    // filter
  filter_config.FilterID2 = 0;//0b1111;                   // mask

  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config);

  HAL_FDCAN_Start(&hfdcan1);

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_TIM_Base_Start_IT(&htim2);

  uart_rx_data_pending = 0U;
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 10);
}

void APP_main() {
  uart_tx_buffer[0] = 0xAAU;  // Start of frame

  uart_tx_buffer[1] = 0x00U;  // Timestamp
  uart_tx_buffer[2] = 0x00U;
  uart_tx_buffer[3] = 0x00U;
  uart_tx_buffer[4] = 0x00U;

  uart_tx_buffer[5] = 0x03U;  // DLC

  uart_tx_buffer[6] = 0x03U;  // ID
  uart_tx_buffer[7] = 0x00U;
  uart_tx_buffer[8] = 0x00U;
  uart_tx_buffer[9] = 0x00U;

  uart_tx_buffer[10] = 0x01U;
  uart_tx_buffer[11] = 0x02U;
  uart_tx_buffer[12] = 0x03U;

  uart_tx_buffer[13] = 0xBBU;  // End of frame


  HAL_UART_Transmit_IT(&huart2, uart_tx_buffer, 14);
  HAL_Delay(1000);
}
