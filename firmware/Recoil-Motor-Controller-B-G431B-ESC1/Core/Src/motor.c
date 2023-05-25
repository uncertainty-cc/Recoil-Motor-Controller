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
  motor->flux_angle_offset = 0;

  // ID test2: 4.533934;
  // ID test3: 4.365709  4.327361

  // ID 2: 6.113920

  // ID 6: 5.206272
  // ID 8: 4.34   4.17
  // ID 10: 3.08



  return HAL_OK;
}

