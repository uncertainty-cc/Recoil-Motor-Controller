/*
 * motor.c
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#include "motor.h"


HAL_StatusTypeDef Motor_init(Motor *motor) {
  motor->pole_pairs = 14;
  motor->kv_rating = 150;

  motor->phase_order = -1;
  motor->flux_angle_offset = 4.533934;


  return HAL_OK;
}

