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
#include "motor_profiles.h"


/**
 * @brief Motor object.
 */
typedef struct {
  uint32_t pole_pairs;
  float    torque_constant;
  int32_t  phase_order;
  float    max_calibration_current;
} Motor;


/**
 * @brief Initialize the Motor instance with default values.
 *
 * @param controller Pointer to the Motor struct.
 * @return Status of the initialization process. HAL_OK if successful.
 */
HAL_StatusTypeDef Motor_init(Motor *motor);

#endif /* INC_MOTOR_H_ */
