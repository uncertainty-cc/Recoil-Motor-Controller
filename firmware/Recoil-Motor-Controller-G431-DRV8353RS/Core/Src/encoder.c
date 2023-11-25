/*
 * encoder.c
 *
 *  Created on: Oct 13, 2022
 *      Author: TK
 */

#include "encoder.h"


HAL_StatusTypeDef Encoder_init(Encoder *encoder, SPI_HandleTypeDef *hspi) {
  encoder->hspi = hspi;

  encoder->cpr = -1 * (1 << 14);  // 14 bit precision

  encoder->position_offset = 0.f;
  encoder->filter_bandwidth = ENCODER_FILTER_BANDWIDTH;

  encoder->position_raw = 0;
  encoder->n_rotations = 0;

  encoder->position = 0.f;
  encoder->velocity = 0.f;

  Encoder_resetFluxOffset(encoder);

  return HAL_OK;
}

void Encoder_setFilterGain(Encoder *encoder, float bandwitdth) {
  encoder->filter_alpha = clampf(1.f - pow(M_E, -2.f * M_PI * (bandwitdth / (float)ENCODER_UPDATE_FREQ)), 0.f, 1.f);
}

void Encoder_resetFluxOffset(Encoder *encoder) {
  encoder->n_rotations = 0;
  encoder->flux_offset = 0.f;
  memset((uint8_t *)encoder->flux_offset_table, 0, 128*sizeof(float));
}

void Encoder_update(Encoder *encoder) {
  encoder->spi_tx_buffer = 0x3FFF | (1 << 14);
  encoder->spi_tx_buffer |= getParity(encoder->spi_tx_buffer) << 15;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
  HAL_SPI_TransmitReceive(encoder->hspi, (uint8_t *)&encoder->spi_tx_buffer, (uint8_t *)&encoder->spi_rx_buffer, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

  // reading is center aligned with range [-cpr/2, cpr/2)
  int16_t reading = READ_BITS(encoder->spi_rx_buffer, 0x3FFF) - abs(encoder->cpr / 2);

  // handle multi-rotation crossing
  int16_t reading_delta = encoder->position_raw - reading;
  if (abs(reading_delta) > abs(encoder->cpr / 2)) {
    encoder->n_rotations += ((encoder->cpr * reading_delta) > 0) ? 1 : -1;
  }
  encoder->position_raw = reading;

  float position = (((float)reading / (float)encoder->cpr) + encoder->n_rotations) * (M_2PI_F);

  float delta_position_filtered = encoder->filter_alpha * (position - encoder->position);
  encoder->position += delta_position_filtered;

  float velocity = delta_position_filtered * (float)ENCODER_UPDATE_FREQ;
  float delta_velocity_filtered = encoder->filter_alpha * (velocity - encoder->velocity);
  encoder->velocity += delta_velocity_filtered;
}

