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

  encoder->cpr = ENCODER_DIRECTION * (1 << ENCODER_PRECISION_BITS);  // 12 bit precision

  encoder->position_offset = 0.f;
  encoder->filter_bandwidth = ENCODER_FILTER_BANDWIDTH;

  encoder->position_raw = 0;
  encoder->n_rotations = 0;

  encoder->position = 0.f;
  encoder->velocity = 0.f;

  Encoder_resetFluxOffset(encoder);

  HAL_StatusTypeDef status = HAL_ERROR;
  while (status) {
    HAL_I2C_Init(encoder->hi2c);

    // wait for I2C device to power up
    HAL_Delay(100);

    status = HAL_I2C_Mem_Read(encoder->hi2c, AS5600_I2C_ADDR << 1, AS5600_ANGLE_ADDR, I2C_MEMADD_SIZE_8BIT, encoder->i2c_buffer, 2, 100);
  }

  return status;
}

void Encoder_setFilterGain(Encoder *encoder, float bandwitdth) {
  encoder->filter_alpha = clampf(1.f - pow(M_E, -2.f * M_PI * (bandwitdth / (float)ENCODER_UPDATE_FREQ)), 0.f, 1.f);
}

void Encoder_resetFluxOffset(Encoder *encoder) {
  encoder->n_rotations = 0;
  encoder->flux_offset = 0.f;
  memset((uint8_t *)encoder->flux_offset_table, 0, ENCODER_LUT_ENTRIES*sizeof(float));
}

void Encoder_update(Encoder *encoder) {
  // 20 kHz commutation cycle is faster than I2C transfer speed (~13.44kHz), so we need to throttle here
  encoder->i2c_update_counter += 1;

  if (encoder->i2c_update_counter == (COMMUTATION_FREQ / ENCODER_UPDATE_FREQ)) {
    encoder->i2c_update_counter = 0;
    HAL_I2C_Master_Receive_IT(encoder->hi2c, AS5600_I2C_ADDR << 1, encoder->i2c_buffer, 2);
  }

  // Read the raw reading from the I2C sensor and center-align it within the range [-cpr/2, cpr/2).
  int16_t reading = ((int16_t)((encoder->i2c_buffer[0]) << 8) | encoder->i2c_buffer[1]) - abs(encoder->cpr / 2);

  // TODO: implement encoder lut-table Linearization
//  /* Linearization */
//  int off_1 = encoder->offset_lut[(encoder->raw)>>9];       // lookup table lower entry
//  int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];   // lookup table higher entry
//  int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
//  encoder->count = encoder->raw + off_interp;

  // Calculate the change in reading
  int16_t reading_delta = encoder->position_raw - reading;

  // Handle multi-rotation crossing.
  if (abs(reading_delta) > abs(encoder->cpr / 2)) {
    encoder->n_rotations += ((encoder->cpr * reading_delta) > 0) ? 1 : -1;
  }
  encoder->position_raw = reading;

  // Convert the raw position to position in radians (rad)
  float position = (((float)reading / (float)encoder->cpr) + encoder->n_rotations) * (M_2PI_F);
  // TODO: implement encoder lut-table Linearization
//                  + encoder->flux_offset_table[reading >> 5];

  // Update the delta position
  float delta_position = position - encoder->position;
  encoder->position = position;

  // Update the filtered velocity
  float velocity = delta_position * (float)ENCODER_UPDATE_FREQ;
  encoder->velocity += encoder->filter_alpha * (velocity - encoder->velocity);
}
