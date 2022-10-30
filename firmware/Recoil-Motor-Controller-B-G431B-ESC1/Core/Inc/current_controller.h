/*
 * foc.h
 *
 *  Created on: Aug 2, 2022
 *      Author: TK
 */

#ifndef INC_CURRENT_CONTROLLER_H_
#define INC_CURRENT_CONTROLLER_H_

#include <stdint.h>

#include "motor_controller_conf.h"
#include "foc_math.h"

typedef struct {
  // parameters
  float current_filter_alpha;

  float i_q_kp;
  float i_q_ki;

  float i_d_kp;
  float i_d_ki;

  // variables
  float i_a_measured;
  float i_b_measured;
  float i_c_measured;

  float i_alpha_measured;
  float i_beta_measured;

  float i_q_measured;
  float i_d_measured;
  float i_q_target;
  float i_d_target;
  float i_q_setpoint;
  float i_d_setpoint;

  float i_q_intergrator;
  float i_d_intergrator;

  float v_q_target;
  float v_d_target;
  float v_q_setpoint;
  float v_d_setpoint;

  float v_alpha_target;
  float v_beta_target;
  float v_alpha_setpoint;
  float v_beta_setpoint;

  float v_a_target;
  float v_b_target;
  float v_c_target;
  float v_a_setpoint;
  float v_b_setpoint;
  float v_c_setpoint;
} CurrentController;

void CurrentController_init(CurrentController *controller);

void CurrentController_update(CurrentController *controller, Mode mode, float sin_theta, float cos_theta, float v_bus);

#endif /* INC_COMMUTATION_CONTROLLER_H_ */
