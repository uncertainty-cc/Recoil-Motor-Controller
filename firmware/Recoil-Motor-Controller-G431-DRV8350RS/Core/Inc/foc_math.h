/*
 * foc.h
 *
 *  Created on: Aug 2, 2022
 *      Author: TK
 */

#ifndef INC_FOC_MATH_H_
#define INC_FOC_MATH_H_

#include <stdint.h>
#include <math.h>

#define SET_BITS(REG, BIT)                    ((REG) |= (BIT))
#define CLEAR_BITS(REG, BIT)                  ((REG) &= ~(BIT))
#define READ_BITS(REG, BIT)                   ((REG) & (BIT))
#define WRITE_BITS(REG, CLEARMASK, SETMASK)   ((REG) = (((REG) & (~(CLEARMASK))) | (SETMASK)))

#define M_E_F         2.7182818284590452354f
#define M_LOG2E_F     1.4426950408889634074f
#define M_LOG10E_F    0.43429448190325182765f
#define M_LN2_F       _M_LN2
#define M_LN10_F      2.302585092994046f
#define M_PI_F        3.141592653589793f
#define M_2PI_F       6.283185307179586f
#define M_PI_2_F      1.5707963267948966f
#define M_PI_4_F      0.7853981633974483f

#define Q31_TO_FLOAT(x)                       ((float)(x) / (float)(0x80000000))
#define FLOAT_TO_Q31(x)                       ((int32_t)((float)(x) * (float)0x7FFFFFFF))

static inline int32_t max(int32_t a, int32_t b) {
  return a > b ? a : b;
}

static inline int32_t min(int32_t a, int32_t b) {
  return a < b ? a : b;
}

static inline float clampf(float value, float min, float max) {
  return (value > max) ? max : ((value < min) ? min : value);
}

static inline float wrapTo2Pi(float value) {
  value = fmodf(value, M_2PI_F);
  return value >= 0.f ? value : (value + M_2PI_F);
}

static inline float fast_fmaxf(float a, float b) {
  return a > b ? a : b;
}

static inline float fast_fminf(float a, float b) {
  return a < b ? a : b;
}

static inline float fast_fmaxf3(float a, float b, float c) {
  return (a > b ? (a > c ? a : c) : (b > c ? b : c));
}

static inline float fast_fminf3(float a, float b, float c) {
  return (a < b ? (a < c ? a : c) : (b < c ? b : c));
}

void FOC_clarkTransform(float *i_alpha, float *i_beta, float i_a, float i_b, float i_c);

void FOC_parkTransform(float *i_q, float *i_d, float i_alpha, float i_beta, float sin_theta, float cos_theta);

void FOC_invParkTransform(float *v_alpha, float *v_beta, float v_q, float v_d, float sin_theta, float cos_theta);

void FOC_invClarkSVPWM(float *v_a, float *v_b, float *v_c, float v_alpha, float v_beta);

#endif /* INC_FOC_MATH_H_ */
