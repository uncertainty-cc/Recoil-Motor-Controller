/*
 * app.c
 *
 *  Created on: Jan 24, 2023
 *      Author: TK
 */

#include "app.h"

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

char str[256];

CAN_Frame can_rx_frame;
CAN_Frame can_tx_frame;

uint8_t uart_rx_buffer[64];
uint8_t uart_rx_data_pending = 0U;

uint8_t uart_tx_buffer[64];

/**
 * CAN receive interrupt routine.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_Frame rx_frame;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  CAN_getRxFrame(&hcan1, &rx_frame);

  uart_tx_buffer[0] = PYTHONCAN_START_OF_FRAME;

  uart_tx_buffer[1] = 0x00U;  // Timestamp
  uart_tx_buffer[2] = 0x00U;
  uart_tx_buffer[3] = 0x00U;
  uart_tx_buffer[4] = 0x00U;

  uart_tx_buffer[5] = rx_frame.size;  // DLC

  uart_tx_buffer[6] = READ_BITS(rx_frame.id, 0xFFU);  // ID
  uart_tx_buffer[7] = READ_BITS(rx_frame.id >> 8U, 0xFFU);
  uart_tx_buffer[8] = READ_BITS(rx_frame.id >> 16U, 0xFFU);
  uart_tx_buffer[9] = READ_BITS(rx_frame.id >> 24U, 0xFFU);

  for (uint16_t i=0; i<rx_frame.size; i+=1) {
    uart_tx_buffer[10+i] = rx_frame.data[i];
  }

  uart_tx_buffer[10+rx_frame.size] = PYTHONCAN_END_OF_FRAME;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  HAL_UART_Transmit_IT(&huart2, uart_tx_buffer, 11+rx_frame.size);
}

/**
 * When UART has not been transmitting for a while, we reset the receive interrupt handler.
 * This is to handle the case where UART is missing data, and we just drop that packet instead of
 * continue receiving the data forever.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  HAL_UART_AbortReceive(&huart2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 11);
}

/**
 * UART receive interrupt routine.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  if (!uart_rx_data_pending) {
    // if we are receiving the header section

    // check if the first byte is the correct Start of Frame
    uint8_t is_valid_frame = uart_rx_buffer[0] == 0xAAU;
    if (!is_valid_frame) {
      // if not, discard and continue receiving
      HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 11);
      return;
    }

    // decode the header section
    can_tx_frame.id_type = CAN_ID_STANDARD;
    can_tx_frame.frame_type = CAN_FRAME_DATA;
//    uint32_t timestamp = ((uart_rx_buffer[1])     // timestamp is not used
//        | (uart_rx_buffer[2] << 8U)
//        | (uart_rx_buffer[3] << 16U)
//        | (uart_rx_buffer[4] << 24U));
    can_tx_frame.size = uart_rx_buffer[5];
    can_tx_frame.id = ((uart_rx_buffer[6])
        | (uart_rx_buffer[7] << 8U)
        | (uart_rx_buffer[8] << 16U)
        | (uart_rx_buffer[9] << 24U));

    // if DLC > 0, we need to continue receive `DLC` number of data
    if (can_tx_frame.size) {
      uart_rx_data_pending = 1U;
      can_tx_frame.data[0] = uart_rx_buffer[10];
      HAL_UART_Receive_IT(&huart2, uart_rx_buffer, can_tx_frame.size);
      return;
    }
  }
  else {
    memcpy(can_tx_frame.data+1, uart_rx_buffer, can_tx_frame.size-1);
  }

  CAN_putTxFrame(&hcan1, &can_tx_frame);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  uart_rx_data_pending = 0U;
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 11);
}

void APP_init() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  uint32_t filter_id = 0;
  uint32_t filter_mask = 0x0;

  // we are receiving all data on the bus, so setting filter and mask to 0 here.
  CAN_FilterTypeDef filter_config;
  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter_config.FilterIdHigh = filter_id << 5;
  filter_config.FilterIdLow = 0;
  filter_config.FilterMaskIdHigh = filter_mask << 5;
  filter_config.FilterMaskIdLow = 0;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterActivation = CAN_FILTER_ENABLE;
  filter_config.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &filter_config);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  uart_rx_data_pending = 0U;
  HAL_UART_Receive_IT(&huart2, uart_rx_buffer, 11);

  HAL_TIM_Base_Start_IT(&htim2);
}

void APP_main() {
  // do nothing in main loop
}
