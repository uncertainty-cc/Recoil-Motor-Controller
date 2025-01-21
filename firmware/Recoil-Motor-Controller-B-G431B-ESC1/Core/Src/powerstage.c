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

  // defaults to be 1000 Hz cutoff, out of 20 kHz loop
  powerstage->bus_voltage_filter_alpha = 0.2695973f;

  powerstage->bus_voltage_measured = NOMINAL_BUS_VOLTAGE;

  PowerStage_disablePWM(powerstage);

  return HAL_OK;
}

HAL_StatusTypeDef PowerStage_start(PowerStage *powerstage) {
  HAL_StatusTypeDef status = HAL_OK;
  status |= HAL_TIM_Base_Start_IT(powerstage->htim);
  status |= HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_1);
  status |= HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_1);
  status |= HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_2);
  status |= HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_2);
  status |= HAL_TIM_PWM_Start(powerstage->htim, TIM_CHANNEL_3);
  status |= HAL_TIMEx_PWMN_Start(powerstage->htim, TIM_CHANNEL_3);

  return status;
}

void PowerStage_setOutputPWM(PowerStage *powerstage, uint16_t ccr_a, uint16_t ccr_b, uint16_t ccr_c, int8_t phase_order) {
  if (phase_order == +1) {
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_3, ccr_c);
  }
  else if (phase_order == -1) {
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_1, ccr_c);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_3, ccr_a);
  }
  else {
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(powerstage->htim, TIM_CHANNEL_3, 0);
  }
}

void PowerStage_setOutputVoltage(PowerStage *powerstage, float v_a, float v_b, float v_c, int8_t phase_order) {
  // Normalize the input voltages to range [0 ~ 1]
  // i.e. convert to PWM duty cycles.
  v_a = .5f * ((v_a / powerstage->bus_voltage_measured) + 1.f);
  v_b = .5f * ((v_b / powerstage->bus_voltage_measured) + 1.f);
  v_c = .5f * ((v_c / powerstage->bus_voltage_measured) + 1.f);

  // Clamp the voltage range to prevent high-side switching bootstrap circuit loses voltage
  // and also allow time for low-side current sampling
  v_a = clampf(v_a, 0.02f, 0.98f);
  v_b = clampf(v_b, 0.02f, 0.98f);
  v_c = clampf(v_c, 0.02f, 0.98f);

  // Calculate the CCR values based on the normalized voltages.
  uint16_t ccr_a = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_a);
  uint16_t ccr_b = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_b);
  uint16_t ccr_c = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(powerstage->htim)+1) * v_c);

  PowerStage_setOutputPWM(powerstage, ccr_a, ccr_b, ccr_c, phase_order);
}

void PowerStage_calibratePhaseCurrentOffset(PowerStage *powerstage) {
  int32_t adc_reading_0 = 0;
  int32_t adc_reading_1 = 0;
  int32_t adc_reading_2 = 0;

  // sample for 100 ms
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

void PowerStage_updateBusVoltage(PowerStage *powerstage) {
  powerstage->bus_voltage_measured += powerstage->bus_voltage_filter_alpha
      * ((HAL_ADCEx_InjectedGetValue(powerstage->hadc1, ADC_INJECTED_RANK_2) * ADC_BUS_VOLTAGE_COEFFICIENT) - powerstage->bus_voltage_measured);
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
  else if (phase_order == -1) {
    *i_c = phase_current_measured_0;
    *i_b = phase_current_measured_1;
    *i_a = phase_current_measured_2;
  }
  else {
    *i_a = 0.f;
    *i_b = 0.f;
    *i_c = 0.f;
  }
}

