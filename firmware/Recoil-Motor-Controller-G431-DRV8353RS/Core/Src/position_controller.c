/*
 * position_controller.c
 *
 *  Created on: Aug 27, 2022
 *      Author: TK
 */

#include "position_controller.h"

HAL_StatusTypeDef PositionController_init(PositionController *controller) {
  controller->update_counter = 0;

  controller->position_kp = 1.f;
  controller->position_ki = 0.0f;

  controller->velocity_kp = .1f;
  controller->velocity_ki = 0.f;


  controller->torque_limit = .2f;

  controller->velocity_limit = 20.f;

  controller->position_limit_lower = -INFINITY;
  controller->position_limit_upper = INFINITY;

  controller->velocity_setpoint = 0.f;
  controller->position_setpoint = 0.f;

  controller->position_integrator = 0.f;
  controller->velocity_integrator = 0.f;

  return HAL_OK;
}

void PositionController_update(PositionController *controller, Mode mode) {
  controller->update_counter += 1;
  if (controller->update_counter == (COMMUTATION_FREQ / POSITION_UPDATE_FREQ)) {
    controller->update_counter = 0;
  }
  else {
    return;
  }

  if (mode == MODE_POSITION) {
    controller->position_setpoint = clampf(
        controller->position_target,
        controller->position_limit_lower,
        controller->position_limit_upper);

    float position_error = controller->position_setpoint - controller->position_measured;
    float velocity_error = 0.f - controller->velocity_measured;

    controller->position_integrator = clampf(
        controller->position_integrator + controller->position_ki * position_error,
        -controller->torque_limit,
        controller->torque_limit);

    controller->torque_target =
        controller->position_kp * position_error +
        controller->velocity_kp * velocity_error +
        controller->position_integrator;
  }
  else if (mode == MODE_VELOCITY) {
    // TODO: implement velocity controlled loop
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
  else {
    // MODE_TORQUE
    /*
     * user sets `controller->torque_target`
     */
  }

  controller->torque_setpoint = clampf(
      controller->torque_target,
      -controller->torque_limit,
      controller->torque_limit);
}
