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
  SPI_HandleTypeDef *hspi;

  float undervoltage_threshold;
  float overvoltage_threshold;

  uint16_t adc_reading_raw[3];
  int16_t adc_reading_offset[3];

  float bus_voltage_measured;
} PowerStage;

void PowerStage_init(PowerStage *powerstage, TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2, SPI_HandleTypeDef *hspi);

void PowerStage_reset(PowerStage *powerstage);

void PowerStage_start(PowerStage *powerstage);

void PowerStage_disableGateDriver(PowerStage *powerstage);

void PowerStage_enableGateDriver(PowerStage *powerstage);

uint8_t PowerStage_isPWMEnabled(PowerStage *powerstage);

void PowerStage_disablePWM(PowerStage *powerstage);

void PowerStage_enablePWM(PowerStage *powerstage);

ErrorCode PowerStage_updateErrorStatus(PowerStage *powerstage);

void PowerStage_setBridgeOutput(PowerStage *powerstage, float v_a, float v_b, float v_c);

void PowerStage_calibratePhaseCurrentOffset(PowerStage *powerstage);

void PowerStage_updatePhaseCurrent(PowerStage *powerstage, float *i_a, float *i_b, float *i_c);

void PowerStage_updateBusVoltage(PowerStage *powerstage);

#endif /* INC_POWERSTAGE_H_ */
