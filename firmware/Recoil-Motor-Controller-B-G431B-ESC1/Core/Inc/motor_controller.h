/*
 * motor_controller.h
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_CONTROLLER_H_
#define INC_MOTOR_CONTROLLER_H_

#include "stm32g4xx_hal.h"

#include "motor_controller_conf.h"
#include "can.h"
#include "current_controller.h"
#include "encoder.h"
#include "motor_profiles.h"
#include "motor.h"
#include "position_controller.h"
#include "powerstage.h"


#define FLASH_CONFIG_ADDRESS    0x0801F800U  // Bank 1, Page 63
#define FLASH_CONFIG_BANK       FLASH_BANK_1
#define FLASH_CONFIG_PAGE       63


/**
 * @brief MotorController object.
 */
typedef struct {
  uint32_t       device_id;
  uint32_t       firmware_version;

  uint32_t       watchdog_timeout;
  uint32_t       fast_frame_frequency;

  Mode           mode;
  uint32_t       error;

  PositionController  position_controller;
  CurrentController   current_controller;

  PowerStage          powerstage;
  Motor               motor;
  Encoder             encoder;
} MotorController;

/**
 * @brief Get the error status.
 *
 * @param controller Pointer to the MotorController struct.
 * @return The error code.
 */
static inline ErrorCode MotorController_getError(MotorController *controller) {
  return controller->error;
}

/**
 * @brief Clear the error.
 *
 * @param controller Pointer to the MotorController struct.
 */
static inline void MotorController_clearError(MotorController *controller) {
  controller->error = ERROR_NO_ERROR;
}

/**
 * @brief Get the current operating mode of the motor controller.
 *
 * @param controller Pointer to the MotorController struct.
 * @return The current mode.
 */
static inline Mode MotorController_getMode(MotorController *controller) {
  return controller->mode;
}

static inline void MotorController_handleCANRead(MotorController *controller, CAN_Frame *rx_frame, CAN_Frame *tx_frame) {
  uint16_t parameter_id = *((uint16_t *)rx_frame->data);
  if (parameter_id >= sizeof(MotorController)) {
    controller->error |= ERROR_CAN_RX_FAULT;
    return;
  }
  *((uint32_t *)tx_frame->data + 1) = *((uint32_t *)((uint8_t *)controller + parameter_id));
}

static inline void MotorController_handleCANWrite(MotorController *controller, CAN_Frame *rx_frame) {
  uint16_t parameter_id = *((uint16_t *)rx_frame->data);
  if (parameter_id >= sizeof(MotorController)) {
    controller->error |= ERROR_CAN_RX_FAULT;
    return;
  }
  *((uint32_t *)((uint8_t *)controller + parameter_id)) = *((uint32_t *)rx_frame->data + 1);
}

/**
 * @brief Initialize the MotorController instance.
 *
 * @param controller Pointer to the MotorController struct.
 */
void MotorController_init(MotorController *controller);

/**
 * Reset the Motor Controller to the initial state.
 *
 * This function clears all intermediate states and sets various controller
 * parameters and variables to their initial values. It is used between mode
 * switches to reset the controller state.
 *
 * @param controller Pointer to the MotorController struct.
 */
void MotorController_reset(MotorController *controller);

void MotorController_setMode(MotorController *controller, Mode mode);

void MotorController_setFluxAngle(MotorController *controller, float angle_setpoint, float voltage_setpoint);

HAL_StatusTypeDef MotorController_loadConfig(MotorController *controller);

HAL_StatusTypeDef MotorController_storeConfig(MotorController *controller);

void MotorController_update(MotorController *controller);

void MotorController_updateService(MotorController *controller);

void MotorController_runCalibrationSequence(MotorController *controller);

void MotorController_handleCANMessage(MotorController *controller, CAN_Frame *rx_frame);

#endif /* INC_MOTOR_CONTROLLER_H_ */
