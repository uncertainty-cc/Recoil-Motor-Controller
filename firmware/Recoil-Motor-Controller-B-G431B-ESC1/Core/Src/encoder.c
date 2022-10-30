/*
 * encoder.c
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */



#include "encoder.h"


void Encoder_init(Encoder *encoder, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim) {
  encoder->hi2c = hi2c;
  encoder->htim = htim;
  encoder->cpr = 4096;  // 12 bit precision

  encoder->direction = -1;
  encoder->velocity_filter_alpha = 0.02;
  encoder->position_offset = 0;

  encoder->n_rotations = 0;

  HAL_I2C_Mem_Read_IT(encoder->hi2c, 0b0110110<<1, 0x0E, I2C_MEMADD_SIZE_8BIT, encoder->i2c_buffer, 2);
}

float Encoder_getOffset(Encoder *encoder) {
  return encoder->position_offset;
}

void Encoder_setOffset(Encoder *encoder, float offset) {
  encoder->position_offset = offset;
}

void Encoder_triggerUpdate(Encoder *encoder) {
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
  HAL_I2C_Master_Receive_IT(encoder->hi2c, 0b0110110<<1, encoder->i2c_buffer, 2);
}

void Encoder_update(Encoder *encoder) {
  float dt = (float)__HAL_TIM_GET_COUNTER(encoder->htim) / 100000.;
//
//  float dt = 1/4000.;

  uint16_t reading = ((uint16_t)encoder->i2c_buffer[0] << 8) | encoder->i2c_buffer[1];
  float position_relative = encoder->direction * ((float)reading / (float)encoder->cpr) * (2*M_PI);

  float delta_position = position_relative - encoder->position_relative;

  if (fabsf(delta_position) > 0.75 * (2*M_PI)) {
    encoder->n_rotations += (delta_position > 0) ? -1 : 1;

    // unwrap delta pos to correct value for velocity calculation
    delta_position += (delta_position > 0) ? -2*M_PI : 2*M_PI;
  }

  encoder->position_relative = position_relative;
  encoder->position_raw = encoder->position_relative + (encoder->n_rotations * (2*M_PI));
  encoder->position = encoder->position_raw;
  encoder->velocity = (encoder->velocity_filter_alpha * delta_position / dt) + ((1 - encoder->velocity_filter_alpha) * encoder->velocity);
}

float Encoder_getRelativePosition(Encoder *encoder) {
  return encoder->position_relative;
}

float Encoder_getRawPosition(Encoder *encoder) {
  return encoder->position_raw;
}

float Encoder_getPosition(Encoder *encoder) {
  return encoder->position;
//  float dt = (float)__HAL_TIM_GET_COUNTER(encoder->htim) / 1000000.;
//  return encoder->position + encoder->velocity * dt;
}

float Encoder_getVelocity(Encoder *encoder) {
  return encoder->velocity;
}
