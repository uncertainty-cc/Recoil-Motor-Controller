/*
 * position_controller.c
 *
 *  Created on: Aug 27, 2022
 *      Author: TK
 */

#include "position_controller.h"

void PositionController_init(PositionController *controller) {
  controller->position_kp = 0.03;//0.03;
  controller->position_ki = 0;//0;
  controller->position_kd = 0.01;//0.01;
  controller->position_kdd = 0.01;//0.01;

  controller->torque_limit_lower = -0.1;
  controller->torque_limit_upper = 0.1;

  controller->velocity_limit_lower = -20;
  controller->velocity_limit_upper = 20;

  controller->position_limit_lower = -100;
  controller->position_limit_upper = 100;

  controller->acceleration_limit_lower = -20;
  controller->acceleration_limit_upper = 20;
}

void PositionController_update(PositionController *controller) {
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

  controller->position_target = clampf(
      controller->position_target,
      controller->position_limit_lower,
      controller->position_limit_upper);

  controller->position_setpoint =
      controller->position_target - controller->position_measured;

  controller->velocity_target = controller->velocity_measured;

  controller->velocity_target = clampf(
      controller->velocity_target,
      controller->velocity_limit_lower,
      controller->velocity_limit_upper);

  controller->velocity_setpoint =
      controller->velocity_target - controller->velocity_measured;


  controller->torque_target =
      controller->position_kp * controller->position_setpoint
    + controller->position_kd * controller->velocity_setpoint
    + controller->position_kdd * 0;

  controller->torque_target = clampf(
      controller->torque_target,
      controller->torque_limit_lower,
      controller->torque_limit_upper);

  controller->torque_setpoint =
      controller->torque_target - controller->torque_measured;

//  controller->torque_setpoint = clampf(
//      controller->torque_setpoint,
//      controller->torque_limit_lower,
//      controller->torque_limit_upper);
}
