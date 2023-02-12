/*
* position_controller.c
*
*  Created on: Aug 27, 2022
*      Author: TK
*/

#include "position_controller.h"

HAL_StatusTypeDef PositionController_init(PositionController *controller) {
  //  controller->position_kp = 0.f;
  controller->position_kp = 0.1f;
//  controller->position_kp = 0.01f;
  controller->position_ki = 0.0f;

  controller->velocity_kp = 0.005f;
//  controller->velocity_kp = 0.f;
  controller->velocity_ki = 0.f;

//  controller->torque_limit = 1.f;
  controller->torque_limit = 0.05f;

  controller->velocity_limit = 20;

  controller->position_limit_lower = -INFINITY;
  controller->position_limit_upper = 10;

  controller->velocity_setpoint = 0;
  controller->position_setpoint = 0;

  controller->position_integrator = 0;
  controller->velocity_integrator = 0;

  return HAL_OK;
}

void PositionController_update(PositionController *controller, Mode mode) {

  //  acceleration = trajectory_follower(command_position, command_velocity)
  //  control_velocity = command_velocity OR control_velocity + acceleration * dt OR 0.0
  //  control_position = command_position OR control_position + control_velocity * dt
  //  position_error = control_position - feedback_position
  //  velocity_error = control_velocity - feedback_velocity
  //  position_integrator = limit(position_integrator + ki * position_error * dt, ilimit)
  //  torque = position_integrator +
  //           kp * kp_scale * position_error +
  //           kd * kd_scale * velocity_error +
  //           command_torque


  if (mode == MODE_POSITION) {
    controller->position_setpoint = clampf(
        controller->position_target,
        controller->position_limit_lower,
        controller->position_limit_upper);

    float position_error = controller->position_setpoint - controller->position_measured;
    float velocity_error = 0.f - controller->velocity_measured;

    controller->torque_target =
        controller->position_kp * position_error +
        controller->velocity_kp * velocity_error +
        controller->position_integrator;

    controller->position_integrator = clampf(
        controller->position_integrator + controller->position_ki * position_error,
        -controller->torque_limit,
        controller->torque_limit);
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
