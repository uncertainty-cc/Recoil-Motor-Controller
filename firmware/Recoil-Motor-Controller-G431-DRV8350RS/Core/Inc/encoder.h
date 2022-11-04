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
#include "foc_math.h"


typedef struct {
  SPI_HandleTypeDef *hspi;
  TIM_HandleTypeDef *htim;

  uint16_t  spi_tx_buffer;
  uint16_t  spi_rx_buffer;

  int32_t   cpr;
  float     position_offset;      // in range (-inf, inf)
  float     position_filter_alpha;
  float     velocity_filter_alpha;
  float     acceleration_filter_alpha;

  int32_t   n_rotations;
  float     position_relative;    // in range [0, 2PI)
  float     position_raw;         // in range (-inf, inf), without offset

  float     position;             // in range (-inf, inf), with offset
  float     velocity;
  float     acceleration;
} Encoder;


void Encoder_init(Encoder *encoder, SPI_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim);

float Encoder_getOffset(Encoder *encoder);

void Encoder_setOffset(Encoder *encoder, float offset);

void Encoder_triggerUpdate(Encoder *encoder);

void Encoder_update(Encoder *encoder);

float Encoder_getRelativePosition(Encoder *encoder);

float Encoder_getRawPosition(Encoder *encoder);

float Encoder_getPosition(Encoder *encoder);

float Encoder_getVelocity(Encoder *encoder);

float Encoder_getAcceleration(Encoder *encoder);

#endif /* INC_ENCODER_H_ */
