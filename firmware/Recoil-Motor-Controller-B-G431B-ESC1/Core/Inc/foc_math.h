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

typedef struct {
    int8_t integer;
    uint8_t fractional;
} fixed16;

static inline int32_t max(int32_t a, int32_t b) {
  return a > b ? a : b;
}

static inline int32_t min(int32_t a, int32_t b) {
  return a < b ? a : b;
}

static inline float clampf(float value, float min, float max) {
  return (value > max) ? max : ((value < min) ? min : value);
}

/**
 * Wrap a floating-point value to the range [0, 2π).
 *
 * This function takes a floating-point value and wraps it to the interval [0, 2π).
 * If the input value is negative, it will be wrapped to the positive value within
 * the interval [0, 2π).
 *
 * @param value - The input floating-point value to be wrapped.
 *
 * @return The wrapped value within the range [0, 2π).
 */
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

static fixed16 float32ToFixed16(float val) {
    val = (val > 127) ? 127 : ((val < -128) ? -128 : val);
    float fractional = fmodf(val, 1);
    fixed16 result;
    result.integer = (int8_t)val;
    result.fractional = (uint8_t)(fabs(fractional) * 0xFF);
    return result;
}

static float fixed16ToFloat32(fixed16 val) {
    float fractional = ((float)val.fractional) / (float)0xFF;
    return val.integer > 0 ? val.integer + fractional : val.integer - fractional;
}

/**
 * Perform Clark transformation to convert the three-phase currents (i_a, i_b, i_c) to alpha-beta
 * components (i_alpha, i_beta).
 *
 * @param i_alpha Pointer to store the resulting alpha-axis phase current component in Amps (A).
 * @param i_beta  Pointer to store the resulting beta-axis phase current component in Amps (A).
 * @param i_a     The current in phase A in Amps (A).
 * @param i_b     The current in phase B in Amps (A).
 * @param i_c     The current in phase C in Amps (A).
 */
void FOC_clarkTransform(float *i_alpha, float *i_beta, float i_a, float i_b, float i_c);

/**
 * Perform Park transformation to convert the given alpha and beta phase current components
 * (i_alpha, i_beta) in the rotor referenece frame to the corresponding direct and quadrature
 * components (i_q, i_d) in the stator reference frame.
 *
 * @param i_q        Pointer to store the resulting q-axis phase current component in Amps (A).
 * @param i_d        Pointer to store the resulting d-axis phase current component in Amps (A).
 * @param i_alpha    Alpha-axis component of the phase current in Amps (A).
 * @param i_beta     Beta-axis component of the phase current in Amps (A).
 * @param sin_theta  Sine of the rotor angle.
 * @param cos_theta  Cosine of the rotor angle.
 */
void FOC_parkTransform(float *i_q, float *i_d, float i_alpha, float i_beta, float sin_theta, float cos_theta);

/**
 * Perform inverse Park transformation to convert the given direct and quadrature
 * components (v_d and v_q) in the stator referenece frame to the corresponding alpha and beta
 * components (v_alpha and v_beta) in the rotor reference frame.
 *
 * @param v_alpha  Pointer to store the resulting alpha-axis component of the phase voltage in Volts (V).
 * @param v_beta   Pointer to store the resulting beta-axis component of the phase voltage in Volts (V).
 * @param v_q      Quadrature component of the phase voltage in Volts (V).
 * @param v_d      Direct component of the phase voltage in Volts (V).
 * @param sin_theta  Sine of the rotor angle.
 * @param cos_theta  Cosine of the rotor angle.
 */
void FOC_invParkTransform(float *v_alpha, float *v_beta, float v_q, float v_d, float sin_theta, float cos_theta);

/**
 * Perform inverse Clark transformation followed by Space Vector Pulse Width Modulation (SVPWM)
 * to convert the given alpha and beta components (v_alpha and v_beta) of the phase voltages to
 * phase voltages (v_a, v_b, v_c).
 *
 * @param v_a     Pointer to store the resulting phase A voltage in Volts (V).
 * @param v_b     Pointer to store the resulting phase B voltage in Volts (V).
 * @param v_c     Pointer to store the resulting phase C voltage in Volts (V).
 * @param v_alpha Alpha component of the phase voltage in Volts (V).
 * @param v_beta  Beta component of the phase voltage in Volts (V).
 */
void FOC_invClarkSVPWM(float *v_a, float *v_b, float *v_c, float v_alpha, float v_beta);

#endif /* INC_FOC_MATH_H_ */
