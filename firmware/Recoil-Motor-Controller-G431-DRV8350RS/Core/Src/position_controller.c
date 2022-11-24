/*
 * position_controller.c
 *
 *  Created on: Aug 27, 2022
 *      Author: TK
 */

#include "position_controller.h"

void PositionController_init(PositionController *controller) {
  controller->position_kp = 30;
  controller->position_ki = 0;
  controller->velocity_kp = 0.001;
  controller->velocity_ki = 0.000003;

  controller->torque_limit = 0.01;

  controller->velocity_limit = 10;

  controller->position_limit_lower = -INFINITY;
  controller->position_limit_upper = INFINITY;

  controller->velocity_setpoint = 0;
  controller->position_setpoint = 0;

  controller->position_integrator = 0;
  controller->velocity_integrator = 0;
}

void PositionController_update(PositionController *controller, Mode mode) {
  float position_setpoint = controller->position_target;
  position_setpoint = clampf(
      position_setpoint,
      controller->position_limit_lower,
      controller->position_limit_upper);

  controller->position_setpoint = position_setpoint;

  float position_error = controller->position_setpoint - controller->position_measured;

  float velocity_setpoint = controller->position_kp * position_error + controller->velocity_target;

  velocity_setpoint = clampf(
      velocity_setpoint,
      -controller->velocity_limit,
      controller->velocity_limit);
  controller->velocity_setpoint = velocity_setpoint;

  float velocity_error = controller->velocity_setpoint - controller->velocity_measured;

  controller->velocity_integrator += controller->velocity_ki * velocity_error;
  controller->velocity_integrator = clampf(
      controller->velocity_integrator,
      -2 * controller->velocity_limit,
      2 * controller->velocity_limit);

  if (mode != MODE_TORQUE) {
    float torque_setpoint = controller->velocity_kp * velocity_error + controller->velocity_integrator + controller->torque_target;
    torque_setpoint = clampf(
        torque_setpoint,
        -controller->torque_limit,
        controller->torque_limit);

    controller->torque_setpoint = torque_setpoint;
  }
  else {
    controller->torque_setpoint = controller->torque_target;
  }
}
