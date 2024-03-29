/*
 * motor.h
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"
#include "motor_controller_conf.h"

typedef struct {
  uint32_t pole_pairs;
  uint32_t kv_rating;
  int8_t phase_order;
  float phase_resistance;
  float phase_inductance;
} Motor;

HAL_StatusTypeDef Motor_init(Motor *motor);

#endif /* INC_MOTOR_H_ */
