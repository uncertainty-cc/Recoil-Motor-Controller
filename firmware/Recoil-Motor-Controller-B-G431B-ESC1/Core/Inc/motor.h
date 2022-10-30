/*
 * motor.h
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

typedef struct {
  uint32_t pole_pairs;
  uint32_t kv_rating;
  float flux_angle_offset;
} Motor;

void Motor_init(Motor *motor);

#endif /* INC_MOTOR_H_ */
