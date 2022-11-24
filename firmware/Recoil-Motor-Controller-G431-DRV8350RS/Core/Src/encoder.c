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

  encoder->cpr = 1U << 14;  // 14 bit precision
  encoder->direction = -1;

  encoder->position_offset = 0;

  encoder->filter_bandwidth = 100;

  encoder->filter_integral = 0;

  encoder->position_reading = 0;
  encoder->position = 0;
  encoder->velocity = 0;

  encoder->dt = 0.1;

  encoder->n_rotations = 0;
}

void Encoder_setFilterBandwidth(Encoder *encoder, float bandwidth) {
//  encoder->filter_bandwidth = bandwidth;
  encoder->filter_bandwidth = 50;
  float w3db = (1. / 8000.) * 2 * M_PI * encoder->filter_bandwidth;
  encoder->filter_k_p = .5 * (2 * w3db);
  encoder->filter_k_i = .5 * (w3db * w3db);
}

void Encoder_triggerUpdate(Encoder *encoder) {
  encoder->spi_tx_buffer = 0x3FFF;
  encoder->spi_tx_buffer |= 1 << 14;
  encoder->spi_tx_buffer |= getParity(encoder->spi_tx_buffer) << 15;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
  HAL_SPI_TransmitReceive_IT(encoder->hspi, (uint8_t *)&encoder->spi_tx_buffer, (uint8_t *)&encoder->spi_rx_buffer, 1);
}

void Encoder_update(Encoder *encoder) {
  encoder->dt = (float)__HAL_TIM_GET_COUNTER(encoder->htim) / 1000000.;
  __HAL_TIM_SET_COUNTER(encoder->htim, 0);
  float dt = 1 / 4000.;

  int16_t reading = READ_BITS(encoder->spi_rx_buffer, 0x3FFF) - (encoder->cpr / 2);

  // handle multi-rotation crossing
  int16_t reading_delta = encoder->position_reading - reading;
  if (abs(reading_delta) > 0.75 * encoder->cpr) {
    encoder->n_rotations += (reading_delta > 0) ? -1 : 1;
  }

  encoder->position_reading = reading;
  float position_measured = (encoder->direction * ((float)reading / (float)encoder->cpr) * (2*M_PI)
      + encoder->n_rotations * 2 * M_PI
      + encoder->position_offset);
  float position_prev = encoder->position;
  float position_error = position_measured - position_prev;

  encoder->filter_integral += encoder->filter_k_i * position_error;

  float position_tune = encoder->filter_k_p * position_error + encoder->filter_integral;

  encoder->position = position_prev + position_tune;

  // subtract the offset to get "true" revolution
  encoder->position_relative = wrapTo2Pi(encoder->position - encoder->position_offset);

  float delta_position = encoder->position - position_prev;
  if (encoder->dt > 0) {
    encoder->velocity = (delta_position / encoder->dt);
  }
}

float Encoder_getOffset(Encoder *encoder) {
  return encoder->position_offset;
}

void Encoder_setOffset(Encoder *encoder, float offset) {
  encoder->position_offset = offset;
}

float Encoder_getRelativePosition(Encoder *encoder) {
  return encoder->position_relative;
}

float Encoder_getPosition(Encoder *encoder) {
  return encoder->position;
}

float Encoder_getVelocity(Encoder *encoder) {
  return encoder->velocity;
}

