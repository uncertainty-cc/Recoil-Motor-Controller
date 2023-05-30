/*
 * encoder.h
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include <math.h>

#include "stm32g4xx_hal.h"
#include "motor_controller_conf.h"
#include "foc_math.h"

typedef struct {
  I2C_HandleTypeDef *hi2c;

  uint8_t   i2c_buffer[2];
  uint8_t   i2c_update_counter;

  int32_t   cpr;
  float     position_offset;      // in range (-inf, inf)
  float     filter_alpha;
  float     flux_offset;
  float     flux_offset_table[128];

  int16_t   position_raw;         // in range [-cpr/2, cpr/2)
  int32_t   n_rotations;

  float     position;             // in range (-inf, inf), with offset
  float     velocity;
} Encoder;


static inline float Encoder_getPositionOffset(Encoder *encoder) {
  return encoder->position_offset;
}

static inline void Encoder_setPositionOffset(Encoder *encoder, float offset) {
  encoder->position_offset = offset;
}

static inline float Encoder_getPositionMeasured(Encoder *encoder) {
  return encoder->position;
}

static inline float Encoder_getPosition(Encoder *encoder) {
  return encoder->position + encoder->position_offset;
}

static inline float Encoder_getVelocity(Encoder *encoder) {
  return encoder->velocity;
}

HAL_StatusTypeDef Encoder_init(Encoder *encoder, I2C_HandleTypeDef *hi2c);

void Encoder_setFilterBandwidth(Encoder *encoder, float bandwidth);

void Encoder_resetFluxOffset(Encoder *encoder);

void Encoder_update(Encoder *encoder, float dt);

#endif /* INC_ENCODER_H_ */
