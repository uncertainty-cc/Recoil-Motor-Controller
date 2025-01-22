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

  // defaults to be 500 Hz cutoff, out of 10 kHz loop
  encoder->velocity_filter_alpha = 0.2695973f;

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

void Encoder_resetFluxOffset(Encoder *encoder) {
  encoder->n_rotations = 0;
  encoder->flux_offset = 0.f;
  memset((uint8_t *)encoder->flux_offset_table, 0, ENCODER_LUT_ENTRIES*sizeof(float));
}

HAL_StatusTypeDef Encoder_update(Encoder *encoder) {
  // 20 kHz commutation cycle is faster than I2C transfer speed (~13.44kHz), so we need to throttle here
  encoder->i2c_update_counter += 1;

  if (encoder->i2c_update_counter == (COMMUTATION_FREQ / ENCODER_UPDATE_FREQ)) {
    encoder->i2c_update_counter = 0;


    // Read the raw reading from the I2C sensor, the range should be [0, cpr-1].
    // safety check to handle encoder data frame mismatch error
    uint16_t raw_reading = (((uint16_t)encoder->i2c_buffer[0]) << 8) | encoder->i2c_buffer[1];
    if (raw_reading >= abs(encoder->cpr)) {
      return 0x04;
    }
    // TODO: implement encoder lut-table Linearization

    HAL_I2C_Master_Receive_IT(encoder->hi2c, AS5600_I2C_ADDR << 1, encoder->i2c_buffer, 2);


    // Calculate the change in reading
    int16_t reading_delta = encoder->position_raw - raw_reading;

    // Handle multi-rotation crossing.
    if (abs(reading_delta) >= abs(encoder->cpr / 2)) {
      encoder->n_rotations += ((encoder->cpr * reading_delta) > 0) ? 1 : -1;
    }
    encoder->position_raw = raw_reading;

    // Convert the raw position to position in radians (rad)
    float position = (((float)raw_reading / (float)encoder->cpr) + encoder->n_rotations) * (M_2PI_F);

    // Update the delta position
    float delta_position = position - encoder->position;
    encoder->position = position;

    // Update the filtered velocity
    float velocity = delta_position * (float)ENCODER_UPDATE_FREQ;
    encoder->velocity += encoder->velocity_filter_alpha * (velocity - encoder->velocity);
  }

  return HAL_OK;
}
