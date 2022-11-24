/*
 * encoder.h
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32g4xx_hal.h"
#include "foc_math.h"


typedef struct {
  SPI_HandleTypeDef *hspi;
  TIM_HandleTypeDef *htim;

  uint16_t  spi_tx_buffer;
  uint16_t  spi_rx_buffer;

  uint16_t  cpr;
  int8_t    direction;
  float     position_offset;      // in range (-inf, inf)
  float     filter_bandwidth;

  float     filter_k_p;
  float     filter_k_i;
  float     filter_integral;

  float dt;

  int16_t   position_reading;     // in range (-cpr/2, cpr/2)
  int32_t   n_rotations;
  float     position_relative;    // in range [0, 2PI)
  float     position;             // in range (-inf, inf), with offset
  float     velocity;
} Encoder;


void Encoder_init(Encoder *encoder, SPI_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim);

void Encoder_setFilterBandwidth(Encoder *encoder, float bandwidth);

float Encoder_getOffset(Encoder *encoder);

void Encoder_setOffset(Encoder *encoder, float offset);

void Encoder_triggerUpdate(Encoder *encoder);

void Encoder_update(Encoder *encoder);

float Encoder_getRelativePosition(Encoder *encoder);

float Encoder_getRawPosition(Encoder *encoder);

float Encoder_getPosition(Encoder *encoder);

float Encoder_getVelocity(Encoder *encoder);

#endif /* INC_ENCODER_H_ */
