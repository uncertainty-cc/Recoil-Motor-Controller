/*
 * foc.c
 *
 *  Created on: Aug 2, 2022
 *      Author: TK
 */

#include "foc_math.h"

void FOC_clarkTransform(float *i_alpha, float *i_beta, float i_a, float i_b, float i_c) {
  //x = (2.0f * a - b  - c) * (1.0f / 3.0f)
  //y = (b - c) * (1.0f / kSqrt3)
//  float const present_Ialpha = 2.0f/3.0f*motor_current_mA[0]-1.0f/3.0f*(motor_current_mA[1]+motor_current_mA[2]);
//  float const present_Ibeta = 1.0f/sqrt3*(motor_current_mA[1]-motor_current_mA[2]);

//  float i_alpha_raw = i_a + (cosf((2./3.) * M_PI) * i_b) + (cosf((4./3.) * M_PI) * i_c);
//  float i_beta_raw  = sinf((2./3.) * M_PI) * i_b + sinf((4./3.) * M_PI) * i_c;
//
//  *i_alpha = i_alpha_raw * (2.f/3.f);
//  *i_beta  = i_beta_raw * (2.f/3.f);

  *i_alpha = (1. / 3.) * (2. * i_a - i_b - i_c);
  *i_beta = (1. / sqrtf(3.)) * (i_b - i_c);
}


void FOC_parkTransform(float *i_q, float *i_d, float i_alpha, float i_beta, float sin_theta, float cos_theta) {
  *i_q  = -(sin_theta * i_alpha) + (cos_theta * i_beta);
  *i_d  =  (cos_theta * i_alpha) + (sin_theta * i_beta);

}

void FOC_invParkTransform(float *v_alpha, float *v_beta, float v_q, float v_d, float sin_theta, float cos_theta) {
  *v_alpha  = -sin_theta * v_q + cos_theta * v_d;
  *v_beta   =  cos_theta * v_q + sin_theta * v_d;
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


