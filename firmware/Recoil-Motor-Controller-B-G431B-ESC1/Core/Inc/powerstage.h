/*
 * powerstage.h
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#ifndef INC_POWERSTAGE_H_
#define INC_POWERSTAGE_H_


#include <stdint.h>

#include "stm32g4xx_hal.h"

#include "motor_controller_conf.h"
#include "foc_math.h"

typedef struct {
  TIM_HandleTypeDef *htim;
  ADC_HandleTypeDef *hadc1;
  ADC_HandleTypeDef *hadc2;

  uint16_t adc_reading_raw[3];
  int16_t adc_reading_offset[3];

  float undervoltage_threshold;
  float overvoltage_threshold;
  float bus_voltage_filter_alpha;

  float bus_voltage_measured;
} PowerStage;


static inline void PowerStage_disablePWM(PowerStage *powerstage) {
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(powerstage->htim);
}

static inline void PowerStage_enablePWM(PowerStage *powerstage) {
  __HAL_TIM_MOE_ENABLE(powerstage->htim);
}

static inline uint8_t PowerStage_isPWMEnabled(PowerStage *powerstage) {
  return READ_BITS(powerstage->htim->Instance->BDTR, TIM_BDTR_MOE) ? 1 : 0;
}

HAL_StatusTypeDef PowerStage_init(PowerStage *powerstage, TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);

void PowerStage_start(PowerStage *powerstage);

void PowerStage_setOutputPWM(PowerStage *powerstage, uint16_t ccr_a, uint16_t ccr_b, uint16_t ccr_c, int8_t phase_order);

void PowerStage_setOutputVoltage(PowerStage *powerstage, float v_a, float v_b, float v_c, int8_t phase_order);

void PowerStage_calibratePhaseCurrentOffset(PowerStage *powerstage);

void PowerStage_updateBusVoltage(PowerStage *powerstage);

void PowerStage_updatePhaseCurrent(PowerStage *powerstage, float *i_a, float *i_b, float *i_c, int8_t phase_order);

#endif /* INC_POWERSTAGE_H_ */
