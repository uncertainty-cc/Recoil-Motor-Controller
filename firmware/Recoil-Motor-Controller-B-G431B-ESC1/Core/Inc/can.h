/**
 * @file can.h
 * @author -T.K.-
 * @brief Generic CAN driver library.
 * @version 1.0.0
 * @date 2023-05-25
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"

/** @defgroup CAN_IDType CAN ID Type
  * @{
  */
typedef enum {
  CAN_ID_STANDARD   = 0U,   /** 11-bit ID */
  CAN_ID_EXTENDED   = 1U,   /** 29-bit ID */
} CAN_IDType;
/** @} */

/** @defgroup CAN_FrameType CAN Frame Type
  * @{
  */
typedef enum {
  CAN_FRAME_REMOTE  = 0U,   /** Remote frame */
  CAN_FRAME_DATA    = 1U,   /** Data frame */
} CAN_FrameType;
/** @} */


/**
 * @brief CAN frame structure.
 */
typedef struct {
  uint32_t id;
  CAN_IDType id_type;
  CAN_FrameType frame_type;
  uint16_t size;
  uint8_t data[8];
} CAN_Frame;


/**
 * @brief Initializes the CAN peripheral.
 * 
 * @param hfdcan    pointer to an FDCAN_HandleTypeDef structure that contains the configuration
 *                  information for the specified FDCAN.
 * @param id_filter the filter to apply to the CAN ID
 * @param id_mask   the mask to apply to the CAN ID
 * @return          the status of the CAN initialization, either HAL_OK or HAL_ERROR
 */
HAL_StatusTypeDef CAN_init(FDCAN_HandleTypeDef *hfdcan, uint32_t id_filter, uint32_t id_mask);

/**
 * @brief Get an FDCAN frame from the Rx FIFO zone into the message RAM.
 * 
 * @param hfdcan    pointer to an FDCAN_HandleTypeDef structure that contains the configuration
 *                  information for the specified FDCAN.
 * @param rx_frame  pointer to a rx_frame struct where the payload of the Rx frame will be stored.
 */
void CAN_getRxFrame(FDCAN_HandleTypeDef *hfdcan, CAN_Frame *rx_frame);

/**
 * @brief Add a message to the Tx FIFO/Queue and activate the corresponding transmission request.
 * 
 * @param hfdcan    pointer to an FDCAN_HandleTypeDef structure that contains the configuration
 *                  information for the specified FDCAN.
 * @param tx_frame  pointer to a tx_frame struct containing the payload of the Tx frame
 * @return          the status of the CAN transmission, either HAL_OK or HAL_ERROR
 */
HAL_StatusTypeDef CAN_putTxFrame(FDCAN_HandleTypeDef *hfdcan, CAN_Frame *tx_frame);

#endif /* INC_CAN_H_ */
