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
#define FLASH_CONFIG_SIZE       32


/**
 * @brief MotorController object.
 */
typedef struct {
  uint8_t             device_id;
  uint32_t            firmware_version;

  uint16_t            watchdog_timeout;
  uint16_t            fast_frame_frequency;

  Encoder             encoder;
  PowerStage          powerstage;
  Motor               motor;

  CurrentController   current_controller;
  PositionController  position_controller;

  Mode                mode;
  ErrorCode           error;
} MotorController;


/**
 * @brief EEPROMConfig structure.
 */
typedef struct {
  __IO  uint32_t  device_id;
  __IO  uint32_t  firmware_version;
  __IO  uint16_t  watchdog_timeout;
  __IO  uint16_t  fast_frame_frequency;

  __IO  int32_t   encoder_cpr;
  __IO  float     encoder_position_offset;
  __IO  float     encoder_filter_bandwidth;
  __IO  float     encoder_flux_offset;

  __IO  float     powerstage_undervoltage_threshold;
  __IO  float     powerstage_overvoltage_threshold;
  __IO  float     powerstage_bus_voltage_filter_alpha;

  __IO  uint32_t  motor_pole_pairs;
  __IO  uint32_t  motor_kv_rating;
  __IO  int32_t   motor_phase_order;
  __IO  float     motor_phase_resistance;
  __IO  float     motor_phase_inductance;
  __IO  float     motor_max_calibration_current;

  __IO  float     current_controller_i_bandwidth;
  __IO  float     current_controller_i_limit;

  __IO  float     position_controller_gear_ratio;
  __IO  float     position_controller_position_kp;
  __IO  float     position_controller_position_ki;
  __IO  float     position_controller_velocity_kp;
  __IO  float     position_controller_velocity_ki;
  __IO  float     position_controller_torque_limit;
  __IO  float     position_controller_velocity_limit;
  __IO  float     position_controller_position_limit_upper;
  __IO  float     position_controller_position_limit_lower;
} EEPROMConfig;


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

void MotorController_handleCANRead(MotorController *controller, Command command, CAN_Frame *tx_frame);

void MotorController_handleCANWrite(MotorController *controller, Command command, uint8_t *rx_data);

#endif /* INC_MOTOR_CONTROLLER_H_ */
