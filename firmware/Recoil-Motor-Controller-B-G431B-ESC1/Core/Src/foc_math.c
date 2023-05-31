/*
 * foc.c
 *
 *  Created on: Aug 2, 2022
 *      Author: TK
 */

#include "foc_math.h"

void FOC_clarkTransform(float *i_alpha, float *i_beta, float i_a, float i_b, float i_c) {
  *i_alpha = (1.f / 3.f) * (2.f * i_a - i_b - i_c);
  *i_beta = (1.f / sqrtf(3.f)) * (i_b - i_c);
}

void FOC_parkTransform(float *i_q, float *i_d, float i_alpha, float i_beta, float sin_theta, float cos_theta) {
  *i_q  = -(sin_theta * i_alpha) + (cos_theta * i_beta);
  *i_d  =  (cos_theta * i_alpha) + (sin_theta * i_beta);
}

void FOC_invParkTransform(float *v_alpha, float *v_beta, float v_q, float v_d, float sin_theta, float cos_theta) {
  *v_alpha  = -(sin_theta * v_q) + (cos_theta * v_d);
  *v_beta   =  (cos_theta * v_q) + (sin_theta * v_d);
}

void FOC_invClarkSVPWM(float *v_a, float *v_b, float *v_c, float v_alpha, float v_beta) {
  float v_a_phase = v_alpha;
  float v_b_phase = (-.5f * v_alpha) + ((sqrtf(3.f)/2.f) * v_beta);
  float v_c_phase = (-.5f * v_alpha) - ((sqrtf(3.f)/2.f) * v_beta);

  float v_neutral = .5f * (fmaxf(fmaxf(v_a_phase, v_b_phase), v_c_phase) + fminf(fminf(v_a_phase, v_b_phase), v_c_phase));

  *v_a = v_a_phase - v_neutral;
  *v_b = v_b_phase - v_neutral;
  *v_c = v_c_phase - v_neutral;
}
