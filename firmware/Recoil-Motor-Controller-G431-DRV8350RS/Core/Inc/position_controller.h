/*
 * foc.h
 *
 *  Created on: Aug 2, 2022
 *      Author: TK
 */

#ifndef INC_POSITION_CONTROLLER_H_
#define INC_POSITION_CONTROLLER_H_

#include <stdint.h>
#include "motor_controller_conf.h"
#include "foc_math.h"

typedef struct {
  float position_kp;
  float position_ki;
  float position_kd;

  float torque_limit;
  float acceleration_limit;
  float velocity_limit;

  float position_limit_upper;
  float position_limit_lower;

  float torque_target;
  float torque_measured;
  float torque_setpoint;

  float velocity_target;
  float velocity_measured;
  float velocity_setpoint;
  float velocity_measured_prev;

  float position_target;
  float position_measured;
  float position_setpoint;

  float position_integrator;
  float velocity_integrator;

  float decel_distance;
  float accel;
  float dx;
} PositionController;


void PositionController_init(PositionController *controller);

void PositionController_update(PositionController *controller, Mode mode);

#endif /* INC_POSITION_CONTROLLER_H_ */
