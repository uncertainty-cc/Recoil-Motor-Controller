/*
 * encoder.h
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "stm32g4xx_hal.h"
#include "motor_controller_conf.h"
#include "foc_math.h"


typedef struct {
  SPI_HandleTypeDef *hspi;

  uint16_t  spi_tx_buffer;
  uint16_t  spi_rx_buffer;

  int32_t   cpr;
  float     position_offset;      // in range (-inf, inf)
  float     filter_bandwidth;
  float     flux_offset;
  float     flux_offset_table[128];

  float     filter_alpha;

  int16_t   position_raw;         // in range [-cpr/2, cpr/2)
  int32_t   n_rotations;

  float     position;             // in range (-inf, inf), with offset
  float     velocity;
} Encoder;


/**
 * In the case of even parity, for a given set of bits, the bits whose value is 1 are counted.
 * If that count is odd, the parity bit value is set to 1, making the total count of occurrences of 1s in the whole set (including the parity bit) an even number.
 * If the count of 1s in a given set of bits is already even, the parity bit's value is 0.
 *
 * @return true if the number of ones in the data package is even, else false.
 */
static inline uint16_t getParity(uint16_t data) {
  data ^= data >> 8;              // example for 8-bit (this line scales it up to 16 bit)
  data ^= data >> 4;              // ( a b c d e f g h ) xor ( 0 0 0 0 a b c d ) = ( a b c d ae bf cg dh )
  data ^= data >> 2;              // ( a b c d ae bf cg dh ) xor ( 0 0 a b c d ae bf ) = ( a b ac bd ace bdf aceg bdfh )
  data ^= data >> 1;              // ( a b ac bd ace bdf aceg bdfh ) xor ( 0 a b ac bd ace bdf aceg ) = ( a ab abc abcd abcde abcdef abcdefg abcdefgh )
  return data & 1;                // if lsb of data is 0 -> data is even. if lsb of data is 1 -> data is odd.
}

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

HAL_StatusTypeDef Encoder_init(Encoder *encoder, SPI_HandleTypeDef *hi2c);

void Encoder_setFilterGain(Encoder *encoder, float bandwidth);

void Encoder_resetFluxOffset(Encoder *encoder);

void Encoder_update(Encoder *encoder);

#endif /* INC_ENCODER_H_ */
