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


/**
 * @brief CurrentController object.
 */
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


static inline void CurrentController_reset(CurrentController *controller) {
  controller->i_q_integrator = 0.f;
  controller->i_d_integrator = 0.f;
  controller->v_q_setpoint = 0.f;
  controller->v_d_setpoint = 0.f;
  controller->v_alpha_setpoint = 0.f;
  controller->v_beta_setpoint = 0.f;
  controller->v_a_setpoint = 0.f;
  controller->v_b_setpoint = 0.f;
  controller->v_c_setpoint = 0.f;
}

/**
 * @brief Initialize the CurrentController instance with default values.
 *
 * This function initializes a CurrentController struct with default initial current limits,
 * measured currents, bandwidth, and proportional-integral gains for the current control loop.
 *
 * @param controller Pointer to the CurrentController struct.
 * @return Status of the initialization process. HAL_OK if successful.
 */
HAL_StatusTypeDef CurrentController_init(CurrentController *controller);

/**
 * @brief Set the proportional-integral (PI) gains for the current controller.
 *
 * This function calculates and sets the proportional and integral gains (KP and KI)
 * for the current controller based on the provided phase resistance and inductance values.
 *
 * @param controller Pointer to the CurrentController struct.
 * @param phase_resistance Phase resistance value in Ohms (R).
 * @param phase_inductance Phase inductance value in Henry (H).
 */
void CurrentController_setPIGain(CurrentController *controller, float phase_resistance, float phase_inductance);

/**
 * @brief Update the CurrentController state based on the specified mode and inputs.
 *
 * This function updates the state of the current controller according to the specified mode
 * and input values. It performs transformations, calculations, and clamping necessary for
 * controlling a motor's currents using Field-Oriented Control (FOC) techniques.
 *
 * @param controller Pointer to the CurrentController struct.
 * @param mode The operating mode of the controller.
 * @param sin_theta Sine value of the rotor angle in radians (rad).
 * @param cos_theta Cosine value of the rotor angle in radians (rad).
 * @param v_bus The motor bus voltage in Volts (V).
 */
void CurrentController_update(CurrentController *controller, Mode mode,
                              float sin_theta, float cos_theta, float v_bus);

#endif /* INC_COMMUTATION_CONTROLLER_H_ */
