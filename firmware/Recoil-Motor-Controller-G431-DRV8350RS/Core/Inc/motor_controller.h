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

  Mode      mode;
  ErrorCode error;

  uint32_t  firmware_version;
  uint8_t   device_id;
} MotorController;

typedef struct {
  uint32_t  device_id;
  uint32_t  firmware_version;

  int32_t   encoder_cpr;
  float     encoder_position_offset;
  float     encoder_velocity_filter_alpha;

  float     powerstage_undervoltage_threshold;
  float     powerstage_overvoltage_threshold;

  uint32_t  motor_pole_pairs;
  uint32_t  motor_kv_rating;
  float     motor_flux_angle_offset;

  float     current_controller_current_filter_alpha;
  float     current_controller_i_q_kp;
  float     current_controller_i_q_ki;
  float     current_controller_i_d_kp;
  float     current_controller_i_d_ki;

  float     position_controller_position_kp;
  float     position_controller_position_ki;
  float     position_controller_position_kd;
  float     position_controller_torque_limit_upper;
  float     position_controller_torque_limit_lower;
  float     position_controller_velocity_limit_upper;
  float     position_controller_velocity_limit_lower;
  float     position_controller_position_limit_upper;
  float     position_controller_position_limit_lower;
} EEPROMConfig;


void MotorController_init(MotorController *controller);

ErrorCode MotorController_getError(MotorController *controller);

Mode MotorController_getMode(MotorController *controller);

void MotorController_setMode(MotorController *controller, Mode mode);

void MotorController_setFluxAngle(MotorController *controller, float angle_setpoint, float voltage_setpoint);

uint32_t MotorController_storeConfig(MotorController *controller);

void MotorController_loadConfig(MotorController *controller);

float MotorController_getTorque(MotorController *controller);

float MotorController_getVelocity(MotorController *controller);

float MotorController_getPosition(MotorController *controller);

void MotorController_updateCommutation(MotorController *controller, ADC_HandleTypeDef *hadc);

void MotorController_triggerPositionUpdate(MotorController *controller);

void MotorController_updatePositionReading(MotorController *controller);

void MotorController_updatePositionController(MotorController *controller);

void MotorController_updateService(MotorController *controller);

void MotorController_runCalibrationSequence(MotorController *controller);

void MotorController_handleCANMessage(MotorController *controller, CAN_Frame *rx_frame);

#endif /* INC_MOTOR_CONTROLLER_H_ */
