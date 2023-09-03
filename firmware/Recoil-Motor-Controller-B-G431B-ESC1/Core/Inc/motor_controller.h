/*
 * motor_controller.h
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_CONTROLLER_H_
#define INC_MOTOR_CONTROLLER_H_

#include "stm32g4xx_hal.h"

#include "can.h"
#include "motor_controller_conf.h"
#include "encoder.h"
#include "powerstage.h"
#include "motor.h"
#include "current_controller.h"
#include "position_controller.h"

typedef struct {
  Encoder             encoder;
  PowerStage          powerstage;
  Motor               motor;

  CurrentController   current_controller;
  PositionController  position_controller;

  Mode        mode;
  ErrorCode   error;

  uint32_t    firmware_version;
  uint8_t     device_id;
} MotorController;

typedef struct {
  __IO  uint32_t  device_id;
  __IO  uint32_t  firmware_version;

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

  __IO  float     current_controller_i_bandwidth;
  __IO  float     current_controller_i_limit;

  __IO  float     position_controller_position_kp;
  __IO  float     position_controller_position_ki;
  __IO  float     position_controller_velocity_kp;
  __IO  float     position_controller_velocity_ki;
  __IO  float     position_controller_torque_limit;
  __IO  float     position_controller_velocity_limit;
  __IO  float     position_controller_position_limit_upper;
  __IO  float     position_controller_position_limit_lower;
} EEPROMConfig;


static inline float MotorController_getTorque(MotorController *controller) {
  return controller->position_controller.torque_measured;
}

static inline float MotorController_getVelocity(MotorController *controller) {
  return controller->position_controller.velocity_measured;
}

static inline float MotorController_getPosition(MotorController *controller) {
  return controller->position_controller.position_measured;
}

static inline ErrorCode MotorController_getError(MotorController *controller) {
  return controller->error;
}

static inline void MotorController_clearError(MotorController *controller) {
  controller->error = ERROR_NO_ERROR;
}

static inline Mode MotorController_getMode(MotorController *controller) {
  return controller->mode;
}

void MotorController_init(MotorController *controller);

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
