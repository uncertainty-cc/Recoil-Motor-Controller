
#include "current_controller.h"

HAL_StatusTypeDef CurrentController_init(CurrentController *controller) {
  controller->i_limit = 10.f;

  controller->i_a_measured = 0.f;
  controller->i_b_measured = 0.f;
  controller->i_c_measured = 0.f;

  controller->i_alpha_measured = 0.f;
  controller->i_beta_measured = 0.f;

  controller->i_q_measured = 0.f;
  controller->i_d_measured = 0.f;

  controller->i_bandwidth = CURRENT_LOOP_BANDWIDTH;

  controller->i_kp = 0.f;
  controller->i_ki = 0.f;

  return HAL_OK;
}

void CurrentController_setPIGain(CurrentController *controller, float phase_resistance, float phase_inductance) {
  controller->i_kp = controller->i_bandwidth * M_2_PI * phase_inductance;
  controller->i_ki = phase_resistance / phase_inductance;
}

void CurrentController_update(CurrentController *controller, Mode mode, float sin_theta, float cos_theta, float v_bus) {
  FOC_clarkTransform(
      &controller->i_alpha_measured,
      &controller->i_beta_measured,
      controller->i_a_measured,
      controller->i_b_measured,
      controller->i_c_measured);

  FOC_parkTransform(
        &controller->i_q_measured,
        &controller->i_d_measured,
        controller->i_alpha_measured,
        controller->i_beta_measured,
        sin_theta, cos_theta);

  controller->i_q_setpoint = clampf(controller->i_q_target, -controller->i_limit, controller->i_limit);
  controller->i_d_setpoint = clampf(controller->i_d_target, -controller->i_limit, controller->i_limit);

  if (mode != MODE_VQD_OVERRIDE) {
    float i_q_error = controller->i_q_setpoint - controller->i_q_measured;
    float i_d_error = controller->i_d_setpoint - controller->i_d_measured;

    controller->i_q_integrator = clampf(
        controller->i_q_integrator + controller->i_kp * controller->i_ki * i_q_error / (float)COMMUTATION_FREQ, -v_bus, v_bus);
    controller->i_d_integrator = clampf(
        controller->i_d_integrator + controller->i_kp * controller->i_ki * i_d_error / (float)COMMUTATION_FREQ, -v_bus, v_bus);

    controller->v_q_target = controller->i_kp * i_q_error + controller->i_q_integrator;
    controller->v_d_target = controller->i_kp * i_d_error + controller->i_d_integrator;
  }
  else {
    /*
     * user sets `controller->v_q_target` and `controller->v_d_target`
     */
  }

  float k = 1.f;
  // clamp voltage
  if (v_bus > 0.f) {
    // CSVPWM over modulation
    float v_max_sq = v_bus * v_bus * 1.15f;
    float v_norm = (
        (controller->v_q_target * controller->v_q_target)
        + (controller->v_d_target * controller->v_d_target)
        );
    if (v_norm > v_max_sq) {
      k = sqrtf(fabsf(v_norm / v_max_sq));
    }
  }
  controller->v_q_setpoint = k * controller->v_q_target;
  controller->v_d_setpoint = k * controller->v_d_target;


  if (mode != MODE_VALPHABETA_OVERRIDE && mode != MODE_CALIBRATION) {
    // calibration mode needs to override v_alpha_setpoint and v_beta_setpoint
    FOC_invParkTransform(
        &controller->v_alpha_setpoint,
        &controller->v_beta_setpoint,
        controller->v_q_setpoint,
        controller->v_d_setpoint,
        sin_theta, cos_theta);
  }
  else {
    /*
     * user sets `controller->v_alpha_setpoint` and `controller->v_beta_setpoint`
     */
  }

  if (mode != MODE_VABC_OVERRIDE) {
    FOC_invClarkSVPWM(
        &controller->v_a_setpoint,
        &controller->v_b_setpoint,
        &controller->v_c_setpoint,
        controller->v_alpha_setpoint,
        controller->v_beta_setpoint);
  }
  else {
    /*
     * user sets `controller->v_a_setpoint`, `controller->v_b_setpoint`,
     * and `controller->v_c_setpoint`
     */
  }
}

