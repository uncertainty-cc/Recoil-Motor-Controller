/*
 * powerstage.c
 *
 *  Created on: Aug 2, 2022
 *      Author: TK
 */

#include "powerstage.h"


HAL_StatusTypeDef PowerStage_init(PowerStage *powerstage, TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2) {
  powerstage->htim = htim;
  powerstage->hadc1 = hadc1;
  powerstage->hadc2 = hadc2;

  powerstage->enabled = 0;

  powerstage->bus_voltage_filter_alpha = .5f;

  powerstage->bus_voltage_measured = 12.f;

  PowerStage_disablePWM(powerstage);
  return HAL_OK;
}

void PowerStage_start(PowerStage *powerstage) {
  HAL_TIM_Base_Start_IT(powerstage->htim);
  HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_3);
}

void PowerStage_setOutputPWM(PowerStage *powerstage, uint16_t ccr_a, uint16_t ccr_b, uint16_t ccr_c, int8_t phase_order) {
  if (phase_order == 1) {
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_3, ccr_c);
  }
  else {
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_1, ccr_c);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_3, ccr_a);
  }
}

void PowerStage_setOutputVoltage(PowerStage *powerstage, float v_a, float v_b, float v_c, int8_t phase_order) {
  v_a = .5f * ((v_a / powerstage->bus_voltage_measured) + 1.f);  // normalize voltage to range 0 ~ 1
  v_b = .5f * ((v_b / powerstage->bus_voltage_measured) + 1.f);  // i.e. convert to PWM duty cycle.
  v_c = .5f * ((v_c / powerstage->bus_voltage_measured) + 1.f);

  v_a = clampf(v_a, 0.02f, 0.98f);  // prevent hi-side switching bootstrap circuit loses voltage
  v_b = clampf(v_b, 0.02f, 0.98f);  // and also allow current sampling to be functional
  v_c = clampf(v_c, 0.02f, 0.98f);

  uint16_t ccr_a = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_a);
  uint16_t ccr_b = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_b);
  uint16_t ccr_c = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_c);

  PowerStage_setOutputPWM(powerstage, ccr_a, ccr_b, ccr_c, phase_order);
}

void PowerStage_calibratePhaseCurrentOffset(PowerStage *powerstage) {
  powerstage->adc_reading_offset[0] = HAL_ADCEx_InjectedGetValue(powerstage->hadc1, ADC_INJECTED_RANK_1);
  powerstage->adc_reading_offset[1] = HAL_ADCEx_InjectedGetValue(powerstage->hadc2, ADC_INJECTED_RANK_1);
  powerstage->adc_reading_offset[2] = HAL_ADCEx_InjectedGetValue(powerstage->hadc2, ADC_INJECTED_RANK_2);
}

void PowerStage_updateBusVoltage(PowerStage *powerstage) {
  powerstage->bus_voltage_measured += powerstage->bus_voltage_filter_alpha * ((HAL_ADCEx_InjectedGetValue(powerstage->hadc1, ADC_INJECTED_RANK_2) * ADC_BUS_VOLTAGE_COEFFICIENT) - powerstage->bus_voltage_measured);
}

void PowerStage_updatePhaseCurrent(PowerStage *powerstage, float *i_a, float *i_b, float *i_c, int8_t phase_order) {
  powerstage->adc_reading_raw[0] = HAL_ADCEx_InjectedGetValue(powerstage->hadc1, ADC_INJECTED_RANK_1);
  powerstage->adc_reading_raw[1] = HAL_ADCEx_InjectedGetValue(powerstage->hadc2, ADC_INJECTED_RANK_1);
  powerstage->adc_reading_raw[2] = HAL_ADCEx_InjectedGetValue(powerstage->hadc2, ADC_INJECTED_RANK_2);
  float phase_current_measured_0 = -(float)(powerstage->adc_reading_raw[0] - powerstage->adc_reading_offset[0]) * ADC_OPAMP_CURRENT_COEFFICIENT;
  float phase_current_measured_1 = -(float)(powerstage->adc_reading_raw[1] - powerstage->adc_reading_offset[1]) * ADC_OPAMP_CURRENT_COEFFICIENT;
  float phase_current_measured_2 = -(float)(powerstage->adc_reading_raw[2] - powerstage->adc_reading_offset[2]) * ADC_OPAMP_CURRENT_COEFFICIENT;

  // positive is flow into phase
  // negative is flow out of phase
  if (phase_order == 1) {
    *i_a = phase_current_measured_0;
    *i_b = phase_current_measured_1;
    *i_c = phase_current_measured_2;
  }
  else {
    *i_c = phase_current_measured_0;
    *i_b = phase_current_measured_1;
    *i_a = phase_current_measured_2;
  }
}

