/*
 * encoder.c
 *
 *  Created on: Oct 13, 2022
 *      Author: TK
 */

#include "encoder.h"


void Encoder_init(Encoder *encoder, SPI_HandleTypeDef *hspi) {
  encoder->hspi = hspi;

  encoder->cpr = -1 * (1 << 14);  // 14 bit precision

  encoder->position_offset = 0.f;
  Encoder_setFilterBandwidth(encoder, 2e4f / 20e6f);

  encoder->position_raw = 0;
  encoder->n_rotations = 0;

  encoder->position = 0.f;
  encoder->velocity = 0.f;
}

void Encoder_setFilterBandwidth(Encoder *encoder, float bandwidth) {
  encoder->filter_alpha = clampf(1.f - pow(M_E, -2.f * M_PI * bandwidth), 0.f, 1.f);

////  encoder->filter_bandwidth = bandwidth;
//  encoder->filter_bandwidth = 50;
//  float w3db = (1. / 8000.) * 2 * M_PI * encoder->filter_bandwidth;
//  encoder->filter_k_p = .5 * (2 * w3db);
//  encoder->filter_k_i = .5 * (w3db * w3db);
}

void Encoder_update(Encoder *encoder, float dt) {
  encoder->spi_tx_buffer = 0x3FFF | (1 << 14);
  encoder->spi_tx_buffer |= getParity(encoder->spi_tx_buffer) << 15;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
  HAL_SPI_TransmitReceive(encoder->hspi, (uint8_t *)&encoder->spi_tx_buffer, (uint8_t *)&encoder->spi_rx_buffer, 1, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);

  // reading is center aligned with range [-cpr/2, cpr/2)
  int16_t reading = READ_BITS(encoder->spi_rx_buffer, 0x3FFF) - abs(encoder->cpr / 2);

  // handle multi-rotation crossing
  int16_t reading_delta = encoder->position_raw - reading;
  if (abs(reading_delta) > abs(encoder->cpr / 2)) {
    encoder->n_rotations += ((encoder->cpr * reading_delta) > 0) ? 1 : -1;
  }
  encoder->position_raw = reading;

  float position = (((float)reading / (float)encoder->cpr) + encoder->n_rotations) * (M_2PI_F);

  float delta_position = position - encoder->position;
  delta_position *= encoder->filter_alpha;
  encoder->position += delta_position;

  // subtract the offset to get "true" revolution
//  encoder->position_relative = wrapTo2Pi(encoder->position);

  if (dt > 0) {
    encoder->velocity = (delta_position / dt);
  }
}

