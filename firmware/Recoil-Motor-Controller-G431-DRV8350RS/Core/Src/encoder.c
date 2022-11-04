/*
 * encoder.c
 *
 *  Created on: Oct 13, 2022
 *      Author: TK
 */

#include "encoder.h"

/**
 * In the case of even parity, for a given set of bits, the bits whose value is 1 are counted.
 * If that count is odd, the parity bit value is set to 1, making the total count of occurrences of 1s in the whole set (including the parity bit) an even number.
 * If the count of 1s in a given set of bits is already even, the parity bit's value is 0.
 *
 * @return true if the number of ones in the data package is even, else false.
 */
uint16_t getParity(uint16_t data) {
  data ^= data >> 8;              // example for 8-bit (this line scales it up to 16 bit)
  data ^= data >> 4;              // ( a b c d e f g h ) xor ( 0 0 0 0 a b c d ) = ( a b c d ae bf cg dh )
  data ^= data >> 2;              // ( a b c d ae bf cg dh ) xor ( 0 0 a b c d ae bf ) = ( a b ac bd ace bdf aceg bdfh )
  data ^= data >> 1;              // ( a b ac bd ace bdf aceg bdfh ) xor ( 0 a b ac bd ace bdf aceg ) = ( a ab abc abcd abcde abcdef abcdefg abcdefgh )
  return data & 1;                // if lsb of data is 0 -> data is even. if lsb of data is 1 -> data is odd.
}


void Encoder_init(Encoder *encoder, SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim) {
  encoder->hspi = hspi;
  encoder->htim = htim;

  encoder->cpr = -16384;  // 14 bit precision

  encoder->position_filter_alpha = 0.03;
  encoder->velocity_filter_alpha = 0.003;
  encoder->acceleration_filter_alpha = 0.003;
  encoder->position_offset = 0;

  encoder->position_raw = 0;
  encoder->position = 0;
  encoder->velocity = 0;

  encoder->n_rotations = 0;
}

float Encoder_getOffset(Encoder *encoder) {
  return encoder->position_offset;
}

void Encoder_setOffset(Encoder *encoder, float offset) {
  encoder->position_offset = offset;
}

void Encoder_triggerUpdate(Encoder *encoder) {
  encoder->spi_tx_buffer = 0x3FFF;
  encoder->spi_tx_buffer |= 1 << 14;
  encoder->spi_tx_buffer |= getParity(encoder->spi_tx_buffer) << 15;

  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
  HAL_SPI_TransmitReceive_IT(encoder->hspi, (uint8_t *)&encoder->spi_tx_buffer, (uint8_t *)&encoder->spi_rx_buffer, 1);
}

void Encoder_update(Encoder *encoder) {
//  float dt = (float)__HAL_TIM_GET_COUNTER(encoder->htim) / 1000000.;
//
  float dt = 1 / 4000.;

  uint16_t reading = READ_BITS(encoder->spi_rx_buffer, 0x3FFF);
//  uint16_t error = READ_BITS(encoder->spi_rx_buffer, 0x4000);

  float position_relative = ((float)reading / (float)encoder->cpr) * (2*M_PI);

  float delta_position = position_relative - encoder->position_relative;

  if (fabsf(delta_position) > 0.75 * (2*M_PI)) {
    encoder->n_rotations += (delta_position > 0) ? -1 : 1;

    // unwrap delta pos to correct value for velocity calculation
    delta_position += (delta_position > 0) ? -2*M_PI : 2*M_PI;
  }

  encoder->position_relative = position_relative;
  encoder->position_raw += encoder->position_filter_alpha * ((encoder->position_relative + (encoder->n_rotations * (2*M_PI))) - encoder->position_raw);
  encoder->position = encoder->position_raw + encoder->position_offset;
  if (dt > 0) {
    float prev_velocity = encoder->velocity;
    encoder->velocity += encoder->velocity_filter_alpha * ((delta_position / dt) - encoder->velocity);
    float delta_velocity = encoder->velocity - prev_velocity;
    encoder->acceleration += encoder->acceleration_filter_alpha * ((delta_velocity / dt) - encoder->acceleration);
  }
}

float Encoder_getRelativePosition(Encoder *encoder) {
  return encoder->position_relative;
}

float Encoder_getRawPosition(Encoder *encoder) {
  return encoder->position_raw;
}

float Encoder_getPosition(Encoder *encoder) {
  return encoder->position;
}

float Encoder_getVelocity(Encoder *encoder) {
  return encoder->velocity;
}

float Encoder_getAcceleration(Encoder *encoder) {
  return encoder->acceleration;
}
