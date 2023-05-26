/*
 * motor.c
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#include "motor.h"


HAL_StatusTypeDef Motor_init(Motor *motor) {
  motor->pole_pairs = 14;

#ifdef MOTORPROFILE_MAD_M6C12_150KV
  motor->kv_rating = 150;
#endif
#ifdef MOTORPROFILE_MAD_5010_310KV
  motor->kv_rating = 310;
#endif
#ifdef MOTORPROFILE_MAD_5010_110KV
  motor->kv_rating = 110;
#endif

  motor->phase_order = -1;

  return HAL_OK;
}

