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
#include "foc_math.h"
#include "motor_controller_conf.h"


/**
 * @brief Motor object.
 */
typedef struct {
  TIM_HandleTypeDef *htim;
  ADC_HandleTypeDef *hadc1;
  ADC_HandleTypeDef *hadc2;

  uint16_t adc_reading_raw[3];
  uint8_t UNUSED_0[1];
  int16_t adc_reading_offset[3];
  uint8_t UNUSED_1[1];

  float undervoltage_threshold;
  float overvoltage_threshold;
  float bus_voltage_filter_alpha;

  float bus_voltage_measured;
} PowerStage;


/**
 * @brief Disable the powerstage PWM output.
 *
 * @param powerstage Pointer to the PowerStage struct.
 */
static inline void PowerStage_disablePWM(PowerStage *powerstage) {
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(powerstage->htim);
}

/**
 * @brief Enable the powerstage PWM output.
 *
 * @param powerstage Pointer to the PowerStage struct.
 */
static inline void PowerStage_enablePWM(PowerStage *powerstage) {
  __HAL_TIM_MOE_ENABLE(powerstage->htim);
}

/**
 * @brief Get the state of the powerstage PWM output.
 *
 * @param powerstage Pointer to the PowerStage struct.
 * @return The state of the PWM output:
 *         - 0 PWM is disabled.
 *         - 1 PWM is enabled.
 */
static inline uint8_t PowerStage_isPWMEnabled(PowerStage *powerstage) {
  return READ_BITS(powerstage->htim->Instance->BDTR, TIM_BDTR_MOE) ? 1 : 0;
}

/**
 * @brief Initialize the PowerStage instance with default values.
 *
 * @param controller Pointer to the PowerStage struct.
 * @param htim Pointer to the TIM_HandleTypeDef structure that configures the Timer interface.
 * @param hadc1 Pointer to the ADC_HandleTypeDef structure that configures the ADC1 interface.
 * @param hadc2 Pointer to the ADC_HandleTypeDef structure that configures the ADC2 interface.
 * @return Status of the initialization process. HAL_OK if successful.
 */
HAL_StatusTypeDef PowerStage_init(PowerStage *powerstage, TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);

/**
 * @brief Start the powerstage PWM timers.
 *
 * @param controller Pointer to the PowerStage struct.
 */
void PowerStage_start(PowerStage *powerstage);

/**
 * @brief Set the output PWM duty cycles.
 *
 * This function sets the PWM duty cycles for the specified PowerStage based on the
 * given channel compare registers (CCR values) and phase order. The PWM duty cycles
 * determine the power output of the PowerStage.
 *
 * @param controller Pointer to the PowerStage struct.
 * @param ccr_a      The PWM duty cycle value for channel A.
 * @param ccr_b      The PWM duty cycle value for channel B.
 * @param ccr_c      The PWM duty cycle value for channel C.
 * @param phase_order The phase order for setting PWM duty cycles:
 *                   - +1: Set duty cycles in the order A, B, C.
 *                   - -1: Set duty cycles in the order C, B, A.
 *
 * @note The phase order determines which PWM channel corresponds to which phase
 *       in the PowerStage.
 */
void PowerStage_setOutputPWM(PowerStage *powerstage, uint16_t ccr_a, uint16_t ccr_b, uint16_t ccr_c, int8_t phase_order);

/**
 * @brief Set the phase output voltage.
 *
 * This function takes the desired phase voltages (v_a, v_b, v_c) and phase order, converts
 * them into PWM duty cycles, then updates the PowerStage's PWM output accordingly.
 *
 * @param controller Pointer to the PowerStage struct.
 * @param v_a Desired voltage for phase A in Volts (V).
 * @param v_b Desired voltage for phase B in Volts (V).
 * @param v_c Desired voltage for phase C in Volts (V).
 * @param phase_order The phase order for setting PWM duty cycles:
 *                   - +1: Set duty cycles in the order A, B, C.
 *                   - -1: Set duty cycles in the order C, B, A.
 */
void PowerStage_setOutputVoltage(PowerStage *powerstage, float v_a, float v_b, float v_c, int8_t phase_order);

/**
 * @brief Calibrate the phase current offset.
 *
 * This function calibrates the phase current offset by averaging ADC readings from
 * three different channels. It performs multiple readings for each channel and
 * computes the average to determine the offset values for each phase.
 *
 * @note This function is a blocking function that uses HAL_Delay(). It should only be
 * called in the main thread.
 *
 * @param controller Pointer to the PowerStage struct.
 */
void PowerStage_calibratePhaseCurrentOffset(PowerStage *powerstage);

/**
 * @brief Updates bus voltage measurement.
 *
 * This function calculates and updates the bus voltage measurement using an exponential
 * moving average filter. It takes the latest raw ADC reading, converts into voltage,
 * and applies an exponential moving average filter to obtain the bus voltage measurement.
 *
 * @param controller Pointer to the PowerStage struct.
 */
void PowerStage_updateBusVoltage(PowerStage *powerstage);

/**
 * @brief Update phase current measurements.
 *
 * @param powerstage Pointer to the PowerStage structure.
 * @param i_a Pointer to store phase current A in Amps (A).
 * @param i_b Pointer to store phase current B measurement (output).
 * @param i_c Pointer to store phase current C measurement (output).
 * @param phase_order The phase order for setting PWM duty cycles:
 *                   - +1: Set duty cycles in the order A, B, C.
 *                   - -1: Set duty cycles in the order C, B, A.
 */
void PowerStage_updatePhaseCurrent(PowerStage *powerstage, float *i_a, float *i_b, float *i_c, int8_t phase_order);

#endif /* INC_POWERSTAGE_H_ */
