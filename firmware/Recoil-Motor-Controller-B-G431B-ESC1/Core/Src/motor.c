/*
 * motor.c
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#include "motor.h"


HAL_StatusTypeDef Motor_init(Motor *motor) {
  motor->pole_pairs = MOTOR_POLE_PAIRS;
  motor->kv_rating = MOTOR_KV_RATING;
  motor->phase_resistance = MOTOR_PHASE_RESISTANCE;
  motor->phase_inductance = MOTOR_PHASE_INDUCTANCE;
  motor->max_calibration_current = MOTOR_CALIBRATION_CURRENT;

  motor->phase_order = MOTOR_PHASE_ORDER;

  return HAL_OK;
}

