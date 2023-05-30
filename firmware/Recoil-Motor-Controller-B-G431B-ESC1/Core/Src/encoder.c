/*
 * encoder.c
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#include "encoder.h"


HAL_StatusTypeDef Encoder_init(Encoder *encoder, I2C_HandleTypeDef *hi2c) {
  encoder->hi2c = hi2c;
  encoder->i2c_update_counter = 0;

  encoder->cpr = 1 * (1 << 12);  // 12 bit precision

  encoder->position_offset = 0.f;
  Encoder_setFilterBandwidth(encoder, 2e5f / 10e6f);
  Encoder_resetFluxOffset(encoder);

  encoder->position_raw = 0;
  encoder->n_rotations = 0;

  encoder->position = 0.f;
  encoder->velocity = 0.f;

  HAL_Delay(100);

  return HAL_I2C_Mem_Read(encoder->hi2c, 0b0110110<<1, 0x0E, I2C_MEMADD_SIZE_8BIT, encoder->i2c_buffer, 2, 100);
}

void Encoder_setFilterBandwidth(Encoder *encoder, float bandwidth) {
  encoder->filter_alpha = clampf(1.f - pow(M_E, -2.f * M_PI * bandwidth), 0.f, 1.f);
}

void Encoder_resetFluxOffset(Encoder *encoder) {
  encoder->n_rotations = 0;
  encoder->flux_offset = 0.f;
  memset((uint8_t *)encoder->flux_offset_table, 0, 128*sizeof(float));
}


void Encoder_update(Encoder *encoder, float dt) {
  // 20 kHz commutation cycle is faster than I2C transfer speed (~13.44kHz), so we need to throttle here
  encoder->i2c_update_counter += 1;
  if (encoder->i2c_update_counter == (COMMUTATION_FREQ / ENCODER_UPDATE_FREQ)) {
    encoder->i2c_update_counter = 0;
    HAL_I2C_Master_Receive_IT(encoder->hi2c, 0b0110110<<1, encoder->i2c_buffer, 2);
  }
  dt *= COMMUTATION_FREQ / ENCODER_UPDATE_FREQ;


  // reading is center aligned with range [-cpr/2, cpr/2)
  int16_t reading = ((int16_t)((encoder->i2c_buffer[0]) << 8) | encoder->i2c_buffer[1]) - abs(encoder->cpr / 2);
//  int16_t reading = ((int16_t)((encoder->i2c_buffer[0]) << 8) | encoder->i2c_buffer[1]);

//  /* Linearization */
//  int off_1 = encoder->offset_lut[(encoder->raw)>>9];       // lookup table lower entry
//  int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];   // lookup table higher entry
//  int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
//  encoder->count = encoder->raw + off_interp;

  // handle multi-rotation crossing
  int16_t reading_delta = encoder->position_raw - reading;
  if (abs(reading_delta) > abs(encoder->cpr / 2)) {
    encoder->n_rotations += ((encoder->cpr * reading_delta) > 0) ? 1 : -1;
  }
  encoder->position_raw = reading;

  float position = (((float)reading / (float)encoder->cpr) + encoder->n_rotations) * (M_2PI_F);
//                  + encoder->flux_offset_table[reading >> 5];

  float delta_position_filtered = encoder->filter_alpha * (position - encoder->position);
  encoder->position += delta_position_filtered;

  if (dt > 0) {
    encoder->velocity = (delta_position_filtered / dt);
  }
}
