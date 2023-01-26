/*
 * powerstage.c
 *
 *  Created on: Oct 13, 2022
 *      Author: TK
 */

#include "powerstage.h"

void PowerStage_init(PowerStage *powerstage, TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2, SPI_HandleTypeDef *hspi) {
  powerstage->htim = htim;
  powerstage->hadc1 = hadc1;
  powerstage->hadc2 = hadc2;
  powerstage->hspi = hspi;

  powerstage->bus_voltage_measured = 24.;

  PowerStage_disablePWM(powerstage);
  PowerStage_disableGateDriver(powerstage);
}

void PowerStage_reset(PowerStage *powerstage) {
  __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_3, 0U);
}

void PowerStage_start(PowerStage *powerstage) {
  PowerStage_reset(powerstage);
  HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(powerstage->htim);
}

void PowerStage_disableGateDriver(PowerStage *powerstage) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
}

void PowerStage_enableGateDriver(PowerStage *powerstage) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
}

uint8_t PowerStage_isPWMEnabled(PowerStage *powerstage) {
  return READ_BITS(powerstage->htim->Instance->BDTR, TIM_BDTR_MOE) ? 1 : 0;
}

void PowerStage_disablePWM(PowerStage *powerstage) {
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(powerstage->htim);
}

void PowerStage_enablePWM(PowerStage *powerstage) {
  __HAL_TIM_MOE_ENABLE(powerstage->htim);
}

ErrorCode PowerStage_updateErrorStatus(PowerStage *powerstage) {
  uint16_t tx_buffer[2];
  uint16_t rx_buffer[2];

  tx_buffer[0] = (1 << 15) | (0x00 << 11);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
  HAL_SPI_TransmitReceive(powerstage->hspi, (uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

//  tx_buffer[0] = (1 << 15) | (0x01 << 11);
//
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
//  HAL_SPI_TransmitReceive(powerstage->hspi, (uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 1, 100);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

  if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) {
    return ERROR_POWERSTAGE_ERROR;
  }
  return ERROR_NO_ERROR;
}

void PowerStage_setBridgeOutput(PowerStage *powerstage, float v_a, float v_b, float v_c) {
  v_a = .5f * ((v_a / powerstage->bus_voltage_measured) + 1.f);  // normalize voltage to range 0 ~ 1
  v_b = .5f * ((v_b / powerstage->bus_voltage_measured) + 1.f);  // i.e. convert to PWM duty cycle.
  v_c = .5f * ((v_c / powerstage->bus_voltage_measured) + 1.f);

  v_a = clampf(v_a, 0.02f, 0.98f);  // prevent hi-side switching bootstrap circuit loses voltage
  v_b = clampf(v_b, 0.02f, 0.98f);  // and also allow current sampling to be functional
  v_c = clampf(v_c, 0.02f, 0.98f);

  uint16_t ccr_a = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_a);
  uint16_t ccr_b = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_b);
  uint16_t ccr_c = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_c);

  __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_1, ccr_a);
  __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_2, ccr_b);
  __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_3, ccr_c);
}

void PowerStage_calibratePhaseCurrentOffset(PowerStage *powerstage) {
  int32_t adc_reading_0 = 0;
  int32_t adc_reading_1 = 0;
  int32_t adc_reading_2 = 0;
  for (uint16_t i=0; i<10; i+=1) {
    adc_reading_0 += HAL_ADCEx_InjectedGetValue(powerstage->hadc1, ADC_INJECTED_RANK_1);
    adc_reading_1 += HAL_ADCEx_InjectedGetValue(powerstage->hadc2, ADC_INJECTED_RANK_1);
    adc_reading_2 += HAL_ADCEx_InjectedGetValue(powerstage->hadc2, ADC_INJECTED_RANK_2);
    HAL_Delay(10);
  }
  powerstage->adc_reading_offset[0] = adc_reading_0 / 10;
  powerstage->adc_reading_offset[1] = adc_reading_1 / 10;
  powerstage->adc_reading_offset[2] = adc_reading_2 / 10;
}

void PowerStage_updatePhaseCurrent(PowerStage *powerstage, float *i_a, float *i_b, float *i_c) {
  powerstage->adc_reading_raw[0] = HAL_ADCEx_InjectedGetValue(powerstage->hadc1, ADC_INJECTED_RANK_1);
  powerstage->adc_reading_raw[1] = HAL_ADCEx_InjectedGetValue(powerstage->hadc2, ADC_INJECTED_RANK_1);
  powerstage->adc_reading_raw[2] = HAL_ADCEx_InjectedGetValue(powerstage->hadc2, ADC_INJECTED_RANK_2);

  // positive is flow into phase
  // negative is flow out of phase
  *i_a = -(float)(powerstage->adc_reading_raw[0] - powerstage->adc_reading_offset[0]) * ADC_OPAMP_CURRENT_COEFFICIENT;
  *i_b = -(float)(powerstage->adc_reading_raw[1] - powerstage->adc_reading_offset[1]) * ADC_OPAMP_CURRENT_COEFFICIENT;
  *i_c = -(float)(powerstage->adc_reading_raw[2] - powerstage->adc_reading_offset[2]) * ADC_OPAMP_CURRENT_COEFFICIENT;
}

void PowerStage_updateBusVoltage(PowerStage *powerstage) {
  powerstage->bus_voltage_measured += 0.1f * ((HAL_ADCEx_InjectedGetValue(powerstage->hadc1, ADC_INJECTED_RANK_2) * ADC_BUS_VOLTAGE_COEFFICIENT) - powerstage->bus_voltage_measured);
}
