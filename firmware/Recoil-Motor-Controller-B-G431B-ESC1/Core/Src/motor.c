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
  motor->phase_resistance = 0.05735062549544696f;
  motor->phase_inductance = 3.325681588015225e-05f;
#endif
#ifdef MOTORPROFILE_MAD_5010_110KV
  motor->kv_rating = 110;
  motor->phase_resistance = 0.21210899547699139f;
  motor->phase_inductance = 0.0001253253134958695f;
#endif
#ifdef MOTORPROFILE_MAD_5010_310KV
  motor->kv_rating = 310;
  motor->phase_resistance = 0.05735062549544696f;
  motor->phase_inductance = 3.325681588015225e-05f;
#endif
#ifdef MOTORPROFILE_MAD_5010_370KV
  motor->kv_rating = 370;
  motor->phase_resistance = 0.03000304860153151f;
  motor->phase_inductance = 1.0717319302328058e-05f;
#endif

  motor->phase_order = 1;

  return HAL_OK;
}

