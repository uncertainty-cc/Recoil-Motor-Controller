/*
 * position_controller.c
 *
 *  Created on: Aug 27, 2022
 *      Author: TK
 */

#include "position_controller.h"

void PositionController_init(PositionController *controller) {
  //  controller->position_kp = 0.f;
  controller->position_kp = 0.01f;
  controller->position_ki = 0.f;
//  controller->velocity_kp = 0.f;
  controller->velocity_kp = 0.0001f;
  controller->velocity_ki = 0.f;

  controller->torque_limit = 1;

  controller->velocity_limit = 20;

  controller->position_limit_lower = -INFINITY;
  controller->position_limit_upper = INFINITY;

  controller->velocity_setpoint = 0;
  controller->position_setpoint = 0;

  controller->position_integrator = 0;
  controller->velocity_integrator = 0;
}

void PositionController_update(PositionController *controller, Mode mode) {
  if (mode == MODE_POSITION) {
    float position_setpoint = controller->position_target;
    controller->position_setpoint = clampf(
        position_setpoint,
        controller->position_limit_lower,
        controller->position_limit_upper);

    float position_error = controller->position_setpoint - controller->position_measured;
    float velocity_error = 0.f - controller->velocity_measured;

    controller->torque_target =
        controller->position_kp * position_error +
        controller->velocity_kp * velocity_error +
        controller->position_integrator;

    controller->position_integrator = clampf(
        controller->position_integrator + controller->position_ki * position_error, -0.1f, 0.1f);
  }
  else if (mode == MODE_VELOCITY) {
//    float velocity_setpoint = controller->velocity_setpoint;
//
//    velocity_setpoint = clampf(
//        velocity_setpoint,
//        -controller->velocity_limit,
//        controller->velocity_limit);
//    controller->velocity_setpoint = velocity_setpoint;
//
//    float velocity_error = controller->velocity_setpoint - controller->velocity_measured;
//
//    controller->velocity_integrator += controller->velocity_ki * velocity_error;
//    controller->velocity_integrator = clampf(
//        controller->velocity_integrator,
//        -2.f * controller->velocity_limit,
//        2.f * controller->velocity_limit);
  }

  controller->torque_setpoint = clampf(
      controller->torque_target,
      -controller->torque_limit,
      controller->torque_limit);
}
