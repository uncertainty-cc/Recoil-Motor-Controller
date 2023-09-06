/*
 * foc.h
 *
 *  Created on: Aug 2, 2022
 *      Author: TK
 */

#ifndef INC_POSITION_CONTROLLER_H_
#define INC_POSITION_CONTROLLER_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"
#include "foc_math.h"
#include "motor_controller_conf.h"


/**
 * @brief PositionController object.
 */
typedef struct {
  uint8_t update_counter;

  float position_kp;
  float position_ki;
  float velocity_kp;
  float velocity_ki;

  float torque_limit;
  float velocity_limit;
  float position_limit_upper;
  float position_limit_lower;

  float torque_target;
  float torque_measured;
  float torque_setpoint;

  float velocity_target;
  float velocity_measured;
  float velocity_setpoint;

  float position_target;
  float position_measured;
  float position_setpoint;

  float position_integrator;
  float velocity_integrator;
} PositionController;


/**
 * @brief Initialize the PositionController instance with default values.
 *
 * This function initializes a PositionController struct with default initial position, velocity
 * and torque limits, and PID gains for the position control loop.
 *
 * @param controller Pointer to the PositionController struct.
 * @return Status of the initialization process. HAL_OK if successful.
 */
HAL_StatusTypeDef PositionController_init(PositionController *controller);

/**
 * @brief Update the PositionController state based on the specified mode and inputs.
 *
 * This function updates the position controller with the given mode.
 * It calculates the control torque based on the specified control mode
 * (position, velocity, or torque) and the corresponding control parameters.
 *
 * If mode is MODE_POSITION:
 *   - Sets the position setpoint based on the target position and position limits.
 *   - Computes position and velocity errors.
 *   - Integrates the position error and limits the integrator value.
 *   - Calculates the control torque based on position and velocity errors and PID gains.
 *
 * If mode is MODE_VELOCITY (not implemented in this function):
 *   - TODO: Implement velocity control loop.
 *
 * If mode is MODE_TORQUE:
 *   - The user should have previously set the torque target directly in `controller->torque_target`.
 *   - No further calculations are performed in this function for torque control.
 *
 * @param controller Pointer to the PositionController struct.
 * @param mode The operating mode of the controller.
 */
void PositionController_update(PositionController *controller, Mode mode);

#endif /* INC_POSITION_CONTROLLER_H_ */
