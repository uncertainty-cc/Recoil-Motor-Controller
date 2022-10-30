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

  uint8_t enabled;
  float bus_voltage_measured;
  float phase_current_measured[3];
} PowerStage;

void PowerStage_init(PowerStage *powerstage, TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);

uint8_t PowerStage_isEnabled(PowerStage *powerstage);

void PowerStage_disable(PowerStage *powerstage);

void PowerStage_enable(PowerStage *powerstage);

void PowerStage_setBridgeOutput(PowerStage *powerstage, float v_a, float v_b, float v_c);

void PowerStage_calibratePhaseCurrentOffset(PowerStage *powerstage);

void PowerStage_getPhaseCurrentRawReading(PowerStage *powerstage);

void PowerStage_getBusVoltage(PowerStage *powerstage);

void PowerStage_getPhaseCurrent(PowerStage *powerstage, float *i_a, float *i_b, float *i_c);

#endif /* INC_POWERSTAGE_H_ */
