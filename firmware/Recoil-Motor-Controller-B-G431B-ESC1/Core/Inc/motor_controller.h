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

  Mode mode;
  ErrorCode error;

  uint8_t device_id;
  uint32_t firmware_version;
} MotorController;

typedef struct {
  uint32_t  device_id;
  uint32_t  firmware_version;

  int32_t   encoder_cpr;
  float     encoder_position_offset;
  float     encoder_filter_alpha;

  float     powerstage_undervoltage_threshold;
  float     powerstage_overvoltage_threshold;
  float     powerstage_bus_voltage_filter_alpha;

  uint32_t  motor_pole_pairs;
  uint32_t  motor_kv_rating;
  int32_t   motor_phase_order;
  float     motor_flux_angle_offset;

  float     current_controller_i_kp;
  float     current_controller_i_ki;
  float     current_controller_i_limit;

  float     position_controller_position_kp;
  float     position_controller_position_ki;
  float     position_controller_velocity_kp;
  float     position_controller_velocity_ki;
  float     position_controller_torque_limit;
  float     position_controller_velocity_limit;
  float     position_controller_position_limit_upper;
  float     position_controller_position_limit_lower;
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
