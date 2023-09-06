/*
 * foc.h
 *
 *  Created on: Aug 2, 2022
 *      Author: TK
 */

#ifndef INC_CURRENT_CONTROLLER_H_
#define INC_CURRENT_CONTROLLER_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"

#include "foc_math.h"
#include "motor_controller_conf.h"

typedef struct {
  // parameters
  float i_bandwidth;
  float i_limit;

  float i_kp;
  float i_ki;

  // variables
  float i_a_measured;
  float i_b_measured;
  float i_c_measured;
  float v_a_setpoint;
  float v_b_setpoint;
  float v_c_setpoint;

  float i_alpha_measured;
  float i_beta_measured;
  float v_alpha_setpoint;
  float v_beta_setpoint;

  float v_q_target;
  float v_d_target;
  float v_q_setpoint;
  float v_d_setpoint;

  float i_q_target;
  float i_d_target;
  float i_q_measured;
  float i_d_measured;
  float i_q_setpoint;
  float i_d_setpoint;

  float i_q_integrator;
  float i_d_integrator;
} CurrentController;

HAL_StatusTypeDef CurrentController_init(CurrentController *controller);

void CurrentController_setPIGain(CurrentController *controller, float phase_resistance, float phase_inductance);

void CurrentController_update(CurrentController *controller, Mode mode,
                              float sin_theta, float cos_theta, float v_bus);

#endif /* INC_COMMUTATION_CONTROLLER_H_ */
