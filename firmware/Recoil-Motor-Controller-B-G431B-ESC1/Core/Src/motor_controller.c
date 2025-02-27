/*
 * motor_controller.c
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#include "motor_controller.h"


extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart2;


void MotorController_init(MotorController *controller) {
  controller->mode = MODE_DISABLED;
  controller->error = ERROR_NO_ERROR;

  controller->watchdog_timeout = 1000;  // in milliseconds (ms)
  controller->fast_frame_frequency = 0;  // in hertz (Hz)

  controller->device_id = DEVICE_CAN_ID;
  controller->firmware_version = FIRMWARE_VERSION;

  HAL_StatusTypeDef status = HAL_OK;
  uint32_t init_error_step = 0;

//  status |= CAN_init(&hfdcan1, controller->device_id, 0b11111);
  status |= CAN_init(&hfdcan1, 0, 0);
  if (status && !init_error_step) init_error_step = 1;

  status |= Encoder_init(&controller->encoder, &hi2c1);
  if (status && !init_error_step) init_error_step = 2;
  status |= PowerStage_init(&controller->powerstage, &htim1, &hadc1, &hadc2);
  if (status && !init_error_step) init_error_step = 3;
  status |= Motor_init(&controller->motor);
  if (status && !init_error_step) init_error_step = 4;

  status |= CurrentController_init(&controller->current_controller);
  if (status && !init_error_step) init_error_step = 5;
  status |= PositionController_init(&controller->position_controller);
  if (status && !init_error_step) init_error_step = 6;

  status |= MotorController_loadConfig(controller);
  if (status && !init_error_step) init_error_step = 7;

  status |= HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);       // LED PWM timer
  if (status && !init_error_step) init_error_step = 8;

  __HAL_TIM_SET_AUTORELOAD(&htim3, 9999);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  status |= HAL_OPAMP_Start(&hopamp1);
  status |= HAL_OPAMP_Start(&hopamp2);
  status |= HAL_OPAMP_Start(&hopamp3);
  if (status && !init_error_step) init_error_step = 9;

  status |= HAL_ADCEx_InjectedStart(&hadc1);
  status |= HAL_ADCEx_InjectedStart(&hadc2);
  if (status && !init_error_step) init_error_step = 10;

  status |= HAL_TIM_Base_Start_IT(&htim2);    // safety watchdog timer
  status |= HAL_TIM_Base_Start(&htim6);       // time keeper timer
  status |= HAL_TIM_Base_Start_IT(&htim8);    // fast frame timer
  if (status && !init_error_step) init_error_step = 11;

  __HAL_TIM_SET_AUTORELOAD(&htim2, (controller->watchdog_timeout * 10) - 1);
  if (controller->fast_frame_frequency) {
    __HAL_TIM_SET_AUTORELOAD(&htim8, (10000 / (controller->fast_frame_frequency)) - 1);
  }
  else {
    __HAL_TIM_SET_AUTORELOAD(&htim8, (10000 / 100) - 1);
  }

  status |= PowerStage_start(&controller->powerstage);
  if (status && !init_error_step) init_error_step = 12;

  if (status != HAL_OK) {
    SET_BITS(controller->error, ERROR_INITIALIZATION_ERROR);
    MotorController_setMode(controller, MODE_DISABLED);

    __HAL_TIM_SET_AUTORELOAD(&htim3, 999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 2);
    while (1) {
      {
        char str[64];
        sprintf(str, "init error at %u\r\n", init_error_step);
        HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

        HAL_Delay(1000);
        // error loop
      }
    }
  }

  #if !LOAD_ID_FROM_FLASH || !LOAD_CONFIG_FROM_FLASH || !LOAD_CALIBRATION_FROM_FLASH
    MotorController_storeConfig(controller);
  #endif

  // wait ADC and opamp to settle.
  HAL_Delay(100);
  PowerStage_calibratePhaseCurrentOffset(&controller->powerstage);

  // change mode to idle
  MotorController_clearError(controller);
  MotorController_setMode(controller, MODE_IDLE);
}

void MotorController_reset(MotorController *controller) {
  // clear all intermediate states
  // order of the functions here does matter
  PositionController_reset(&controller->position_controller);
  CurrentController_reset(&controller->current_controller);
  PowerStage_setOutputVoltage(&controller->powerstage, 0.f, 0.f, 0.f, controller->motor.phase_order);
}

void MotorController_setMode(MotorController *controller, Mode mode) {
  if (controller->mode == mode) {
    return;
  }

  // because this method could be run in lower priority routines, so
  // it can be interrupted by commutation loop half-way.
  // no matter what state we want to switch to, it's safe to first disable
  // powerstage temporarily when we are performing state switching
  PowerStage_disablePWM(&controller->powerstage);

  switch (mode) {
    case MODE_DISABLED:
      __HAL_TIM_SET_AUTORELOAD(&htim3, 9999);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 8);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      // sleep
      break;

    case MODE_IDLE:
      __HAL_TIM_SET_AUTORELOAD(&htim3, 9999);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 2);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      break;

    case MODE_CALIBRATION:
      __HAL_TIM_SET_AUTORELOAD(&htim3, 1999);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 4);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      MotorController_reset(controller);
      PowerStage_enablePWM(&controller->powerstage);
      break;

    case MODE_DAMPING:
    case MODE_POSITION:
    case MODE_VELOCITY:
    case MODE_TORQUE:
    case MODE_CURRENT:
    case MODE_VQD_OVERRIDE:
    case MODE_VALPHABETA_OVERRIDE:
    case MODE_VABC_OVERRIDE:
      __HAL_TIM_SET_AUTORELOAD(&htim3, 1999);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 2);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      if (controller->mode == MODE_IDLE
        || controller->mode == MODE_DAMPING
        || controller->mode == MODE_POSITION
        || controller->mode == MODE_VELOCITY
        || controller->mode == MODE_TORQUE
        || controller->mode == MODE_CURRENT
        || controller->mode == MODE_VQD_OVERRIDE
        || controller->mode == MODE_VALPHABETA_OVERRIDE
        || controller->mode == MODE_VABC_OVERRIDE) {
        // these are the only allowed state transition
        MotorController_reset(controller);
        PowerStage_enablePWM(&controller->powerstage);
      }
      else {
        // otherwise we set the fault status
//        PowerStage_disable(&controller->powerstage);
        controller->mode = MODE_IDLE;
        SET_BITS(controller->error, ERROR_INVALID_MODE);
        return;  // return directly, do not update mode
      }
      break;

    case MODE_DEBUG:
      break;

    default:
      controller->mode = MODE_IDLE;
      SET_BITS(controller->error, ERROR_INVALID_MODE);
      return;  // return directly, do not update mode
  }
  controller->mode = mode;
}

void MotorController_setFluxAngle(MotorController *controller, float angle_setpoint, float voltage_setpoint) {
  float theta = wrapTo2Pi(angle_setpoint);
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);
  float v_q = 0.f;
  float v_d = voltage_setpoint;

  controller->current_controller.v_alpha_setpoint = -sin_theta * v_q + cos_theta * v_d;
  controller->current_controller.v_beta_setpoint  =  cos_theta * v_q + sin_theta * v_d;
}

HAL_StatusTypeDef MotorController_loadConfig(MotorController *controller) {
  MotorController *controller_config = (MotorController *)FLASH_CONFIG_ADDRESS;

  controller->firmware_version                                  = FIRMWARE_VERSION;

  #if LOAD_CALIBRATION_FROM_FLASH
    if (isnan(controller_config->encoder.flux_offset)) return HAL_ERROR;
    controller->encoder.flux_offset                             = controller_config->encoder.flux_offset;
  #endif
  #if LOAD_ID_FROM_FLASH
    controller->device_id                                       = (uint8_t)controller_config->device_id;
  #endif
  #if LOAD_CONFIG_FROM_FLASH
    controller->watchdog_timeout                                = controller_config->watchdog_timeout;
    controller->fast_frame_frequency                            = controller_config->fast_frame_frequency;
    if (isnan(controller_config->position_controller.gear_ratio))            return HAL_ERROR;
    controller->position_controller.gear_ratio                  = controller_config->position_controller.gear_ratio;
    if (isnan(controller_config->position_controller.torque_filter_alpha))   return HAL_ERROR;
    controller->position_controller.torque_filter_alpha         = controller_config->position_controller.torque_filter_alpha;
    if (isnan(controller_config->position_controller.position_kp))           return HAL_ERROR;
    controller->position_controller.position_kp                 = controller_config->position_controller.position_kp;
    if (isnan(controller_config->position_controller.position_ki))           return HAL_ERROR;
    controller->position_controller.position_ki                 = controller_config->position_controller.position_ki;
    if (isnan(controller_config->position_controller.velocity_kp))           return HAL_ERROR;
    controller->position_controller.velocity_kp                 = controller_config->position_controller.velocity_kp;
    if (isnan(controller_config->position_controller.velocity_ki))           return HAL_ERROR;
    controller->position_controller.velocity_ki                 = controller_config->position_controller.velocity_ki;
    if (isnan(controller_config->position_controller.torque_limit))          return HAL_ERROR;
    controller->position_controller.torque_limit                = controller_config->position_controller.torque_limit;
    if (isnan(controller_config->position_controller.velocity_limit))        return HAL_ERROR;
    controller->position_controller.velocity_limit              = controller_config->position_controller.velocity_limit;
    if (isnan(controller_config->position_controller.position_limit_lower))  return HAL_ERROR;
    controller->position_controller.position_limit_lower        = controller_config->position_controller.position_limit_lower;
    if (isnan(controller_config->position_controller.position_limit_upper))  return HAL_ERROR;
    controller->position_controller.position_limit_upper        = controller_config->position_controller.position_limit_upper;
    if (isnan(controller_config->position_controller.position_offset))       return HAL_ERROR;
    controller->position_controller.position_offset             = controller_config->position_controller.position_offset;

    if (isnan(controller_config->current_controller.i_limit))                return HAL_ERROR;
    controller->current_controller.i_limit                      = controller_config->current_controller.i_limit;
    if (isnan(controller_config->current_controller.i_kp))                   return HAL_ERROR;
    controller->current_controller.i_kp                         = controller_config->current_controller.i_kp;
    if (isnan(controller_config->current_controller.i_ki))                   return HAL_ERROR;
    controller->current_controller.i_ki                         = controller_config->current_controller.i_ki;

    if (isnan(controller_config->powerstage.undervoltage_threshold))         return HAL_ERROR;
    controller->powerstage.undervoltage_threshold               = controller_config->powerstage.undervoltage_threshold;
    if (isnan(controller_config->powerstage.overvoltage_threshold))          return HAL_ERROR;
    controller->powerstage.overvoltage_threshold                = controller_config->powerstage.overvoltage_threshold;
    if (isnan(controller_config->powerstage.bus_voltage_filter_alpha))       return HAL_ERROR;
    controller->powerstage.bus_voltage_filter_alpha             = controller_config->powerstage.bus_voltage_filter_alpha;

    controller->motor.pole_pairs                                = controller_config->motor.pole_pairs;
    if (isnan(controller_config->motor.torque_constant))                     return HAL_ERROR;
    controller->motor.torque_constant                           = controller_config->motor.torque_constant;
    controller->motor.phase_order                               = (int8_t)controller_config->motor.phase_order;
    if (isnan(controller_config->motor.max_calibration_current))             return HAL_ERROR;
    controller->motor.max_calibration_current                   = controller_config->motor.max_calibration_current;

    controller->encoder.cpr                                     = controller_config->encoder.cpr;
    if (isnan(controller_config->encoder.position_offset))                   return HAL_ERROR;
    controller->encoder.position_offset                         = controller_config->encoder.position_offset;
    if (isnan(controller_config->encoder.position_offset))                   return HAL_ERROR;
    controller->encoder.velocity_filter_alpha                   = controller_config->encoder.velocity_filter_alpha;
  #endif

  MotorController_reset(controller);

  return HAL_OK;
}

HAL_StatusTypeDef MotorController_storeConfig(MotorController *controller) {
  FLASH_EraseInitTypeDef erase_init_struct;
  uint32_t page_error;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Erase the user Flash area */
  erase_init_struct.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init_struct.Banks = FLASH_CONFIG_BANK;
  erase_init_struct.Page = FLASH_CONFIG_PAGE;
  erase_init_struct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK) {
    /*Error occurred while page erase.*/
    HAL_FLASH_Lock();
    return HAL_ERROR;
  }

  /* Program the user Flash area word (uint64_t) by word*/
  for (uint16_t i=0; i<FLASH_PAGE_SIZE; i+=sizeof(uint64_t)) {
    volatile uint64_t buf = *((uint64_t *)((uint8_t *)controller + i));

    uint32_t target_address = FLASH_CONFIG_ADDRESS + i;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, target_address, buf) != HAL_OK) {
      HAL_FLASH_Lock();
      return HAL_ERROR;
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return HAL_OK;
}

void MotorController_update(MotorController *controller) {
  // CPU time: 20.3%, 20.6 us maximum total
  // 10kHz refresh rate requirements:
  //  - -O2 optimization
  //  - Release mode
  //  - 10kHz I2C transaction rate
  //  - float32 calculation

  // this is the most time-sensitive
  // takes 1.5 us to run (3%)
  PowerStage_updatePhaseCurrent(&controller->powerstage,
      (float *)(&controller->current_controller.i_a_measured),
      (float *)(&controller->current_controller.i_b_measured),
      (float *)(&controller->current_controller.i_c_measured),
      controller->motor.phase_order);

  // this is also quite time-sensitive
  // if issuing new I2C frame, takes 1.4 us to run (3%)
  // else takes 1.1 us to run (2%)
  HAL_StatusTypeDef status = Encoder_update(&controller->encoder);
  if (status != HAL_OK) {
    controller->error |= ERROR_ENCODER_FAULT;
    MotorController_setMode(&controller, MODE_DAMPING);
  }

  // this block takes 0.5 us to run (1%)
  controller->position_controller.position_measured = Encoder_getPosition(&controller->encoder) / controller->position_controller.gear_ratio;
  controller->position_controller.velocity_measured = Encoder_getVelocity(&controller->encoder) / controller->position_controller.gear_ratio;

  controller->position_controller.torque_measured =
        controller->motor.torque_constant
      * controller->current_controller.i_q_measured
      * controller->position_controller.gear_ratio;

  // takes 1.3 us to run (3%)
  PositionController_update(&controller->position_controller, controller->mode);

  // this block takes 0.5 us to run (1%)
  if (controller->mode == MODE_POSITION
      || controller->mode == MODE_VELOCITY
      || controller->mode == MODE_TORQUE) {
    // same here, the 1.75 magic number...
    controller->current_controller.i_q_target =
        controller->position_controller.torque_setpoint
        / controller->motor.torque_constant
        / controller->position_controller.gear_ratio;
    controller->current_controller.i_d_target = 0.f;
  }
  else {
    // MODE_CURRENT
    /*
     * user sets `controller->i_q_target` and `controller->i_d_target`
     */
  }

  // this block takes 0.7 us to run (1%)
  PowerStage_updateBusVoltage(&controller->powerstage);

  // this block takes 7.3 us maximum to run (15%)
  // 0.0005f is kinda a magic number. Ideally this should be the delay, in seconds, of the encoder signal.
  // if this value is too large, the motor will become stucky at high speed.
  // if this value is too small, the motor cannot reach high speed.
  float theta = wrapTo2Pi(
      ((Encoder_getPositionMeasured(&controller->encoder) + 0.0005f * Encoder_getVelocity(&controller->encoder))
          * (float)controller->motor.pole_pairs)
      - controller->encoder.flux_offset
      );

  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);

  // takes 6 us to run (12%)
  CurrentController_update(&controller->current_controller,
      controller->mode,
      sin_theta,
      cos_theta,
      controller->powerstage.bus_voltage_measured);

  // takes 1.8 us to run (4%)
  if (controller->mode == MODE_DAMPING) {
    PowerStage_setOutputPWM(&controller->powerstage, 0U, 0U, 0U, controller->motor.phase_order);
  }
  else if (controller->mode ==  MODE_CALIBRATION
      || controller->mode == MODE_POSITION
      || controller->mode == MODE_VELOCITY
      || controller->mode == MODE_TORQUE
      || controller->mode == MODE_CURRENT
      || controller->mode == MODE_VQD_OVERRIDE
      || controller->mode == MODE_VALPHABETA_OVERRIDE
      || controller->mode == MODE_VABC_OVERRIDE) {
    PowerStage_setOutputVoltage(&controller->powerstage,
      controller->current_controller.v_a_setpoint,
      controller->current_controller.v_b_setpoint,
      controller->current_controller.v_c_setpoint,
      controller->motor.phase_order);
  }
  else {
    PowerStage_disablePWM(&controller->powerstage);
    PowerStage_setOutputVoltage(&controller->powerstage, 0.f, 0.f, 0.f, controller->motor.phase_order);
  }
}

void MotorController_updateService(MotorController *controller) {
  if (controller->mode == MODE_CALIBRATION) {
    MotorController_runCalibrationSequence(controller);
    return;
  }
}

void MotorController_runCalibrationSequence(MotorController *controller) {
  MotorController_setMode(controller, MODE_CALIBRATION);
  HAL_Delay(10);  // wait for state machine to switch

  // set all calibration data to 0
  // we also reset n_rotation to 0 so Encoder_getPositionMeasured will return value in (-2pi, 2pi)
  Encoder_resetFluxOffset(&controller->encoder);

  // if controller is only powered with VDD, wait for motor power
  while (controller->powerstage.bus_voltage_measured < 9) {
    {
      char str[128];
      sprintf(str, "waiting for bus voltage: %f\r\n", controller->powerstage.bus_voltage_measured);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    }
  }

  // maximum supported number of pole pairs is 32
  float error_table[ENCODER_LUT_ENTRIES * 32];

  // store normal running v_alpha and v_beta values. We need to change this during the calibration.
  float prev_v_alpha_target = controller->current_controller.v_alpha_setpoint;
  float prev_v_beta_target = controller->current_controller.v_beta_setpoint;

  // starting voltage setpoint (V)
  float voltage_setpoint = 0.2f;
  float flux_angle_setpoint = 0.f;

  MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);

  HAL_Delay(500);

  float phase_current = 0.f;

  // gradually ramp up the voltage setpoint until we reach target phase current value.
  while (phase_current < controller->motor.max_calibration_current) {
    HAL_Delay(100);
    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);

    voltage_setpoint += 0.1f;
    phase_current = 1.f/3.f * (
        fabs(controller->current_controller.i_a_measured)
        + fabs(controller->current_controller.i_b_measured)
        + fabs(controller->current_controller.i_c_measured));
    {
      char str[128];
      sprintf(str, "voltage: %f\tphase current: %f\r\n", voltage_setpoint, phase_current);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    }

    // if we cannot even reach the target phase current at more than 10V above nominal bus voltage, there's something wrong. End calibration.
    if (voltage_setpoint > NOMINAL_BUS_VOLTAGE + 10.f) {
      SET_BITS(controller->error, ERROR_CALIBRATION_ERROR);
      MotorController_setMode(controller, MODE_IDLE);
      return;
    }
  }

  HAL_Delay(1000);

  // move one mechanical revolution forward
  for (uint32_t i=0; i<ENCODER_LUT_ENTRIES*controller->motor.pole_pairs; i+=1) {
    flux_angle_setpoint = ((float)i / (ENCODER_LUT_ENTRIES * controller->motor.pole_pairs)) * (2*M_PI) * controller->motor.pole_pairs;

    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);

    float error = Encoder_getPositionMeasured(&controller->encoder) * controller->motor.pole_pairs - flux_angle_setpoint;
    error_table[i] = error;
  }

  HAL_Delay(500);

  // move one mechanical revolution backward
  for (uint32_t i=ENCODER_LUT_ENTRIES*controller->motor.pole_pairs; i>0; i-=1) {
    flux_angle_setpoint = ((float)i / (ENCODER_LUT_ENTRIES * controller->motor.pole_pairs)) * (2*M_PI) * controller->motor.pole_pairs;

    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);

    float error = Encoder_getPositionMeasured(&controller->encoder) * controller->motor.pole_pairs - flux_angle_setpoint;
    error_table[i-1] = 0.5f * (error_table[i-1] + error);
  }

  // release motor
  PowerStage_disablePWM(&controller->powerstage);

  // Calculate average offset
  float flux_offset_sum = 0;
  for (uint32_t i=0; i<ENCODER_LUT_ENTRIES*controller->motor.pole_pairs; i+=1) {
    flux_offset_sum += error_table[i];
  }
//  controller->encoder.flux_offset = wrapTo2Pi(flux_offset_sum / (float)(N_LUT * controller->motor.pole_pairs));
  controller->encoder.flux_offset = flux_offset_sum / (float)(ENCODER_LUT_ENTRIES * controller->motor.pole_pairs);

  // should be 6.178778

  {
    char str[128];
    sprintf(str, "offset angle: %f %f\r\n", flux_offset_sum / (float)(ENCODER_LUT_ENTRIES * controller->motor.pole_pairs), controller->encoder.flux_offset);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }

// Moving average to filter out cogging ripple
  int16_t window = ENCODER_LUT_ENTRIES;
  int16_t lut_offset = ((controller->motor.pole_pairs*M_2PI_F)-error_table[0]) * ENCODER_LUT_ENTRIES / (controller->motor.pole_pairs*M_2PI_F);

  // make sure lut_offset is always >= 0
  if (lut_offset < 0) {
    lut_offset += ENCODER_LUT_ENTRIES;
  }

  {
    char str[128];
    sprintf(str, "lut_offset: %d\r\n", lut_offset);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }

  for (int16_t i=0; i<ENCODER_LUT_ENTRIES; i+=1) {
    float moving_avg = 0;

    for (int16_t j=-window/2; j<window/2; j+=1) {
      int32_t index = (i * controller->motor.pole_pairs) + j;
      // make sure index is always >= 0
      if (index < 0) {
        index += controller->motor.pole_pairs * ENCODER_LUT_ENTRIES;
      }
      // make sure index is always < controller->motor.pole_pairs * ENCODER_LUT_RESOLUTION
      else if (index >= controller->motor.pole_pairs * ENCODER_LUT_ENTRIES) {
        index -= controller->motor.pole_pairs * ENCODER_LUT_ENTRIES;
      }
      moving_avg += error_table[index];
    }

    moving_avg = moving_avg / window;
    int32_t lut_index = lut_offset + i;
//    if (lut_index >= N_LUT) {
//      lut_index -= N_LUT;
//    }
    lut_index = lut_index % ENCODER_LUT_ENTRIES;
    controller->encoder.flux_offset_table[lut_index] = moving_avg - controller->encoder.flux_offset;

    {
      char str[128];
      sprintf(str, "lut_index: %d, %f\r\n", lut_index, moving_avg - controller->encoder.flux_offset);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    }
  }

  {
    char str[128];
    sprintf(str, "done calibration!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }


  controller->current_controller.v_alpha_setpoint = prev_v_alpha_target;
  controller->current_controller.v_beta_setpoint = prev_v_beta_target;

  MotorController_storeConfig(controller);

  HAL_Delay(1000);

  MotorController_setMode(controller, MODE_IDLE);
}

void MotorController_handleCANMessage(MotorController *controller, CAN_Frame *rx_frame) {
  uint16_t device_id = (rx_frame->id) & 0b1111111;
  if (device_id && device_id != controller->device_id) {
    return;
  }

  uint16_t func_id = (rx_frame->id) >> 7;

  CAN_Frame tx_frame;
  tx_frame.id = controller->device_id;
  tx_frame.id_type = CAN_ID_STANDARD;
  tx_frame.frame_type = CAN_FRAME_DATA;
  tx_frame.size = 0;

  switch (func_id) {
    case FUNC_NMT:
      MotorController_handleNMT(controller, rx_frame);
      break;
    
    case FUNC_SYNC_EMCY:
      tx_frame.size = 1;
      *((uint8_t *)tx_frame.data) = controller->device_id;
      break;

    case FUNC_RECEIVE_SDO:
      MotorController_handleSDO(controller, rx_frame, &tx_frame);
      break;

    case FUNC_RECEIVE_PDO_1:
      // echo back the received data
      tx_frame.id = (FUNC_TRANSMIT_PDO_1 << 7) | controller->device_id;
      tx_frame.size = 8;
      *((uint32_t *)tx_frame.data + 0) = *((uint32_t *)rx_frame->data + 0);
      *((uint32_t *)tx_frame.data + 1) = *((uint32_t *)rx_frame->data + 1);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      break;

    case FUNC_RECEIVE_PDO_2:
      // receive [position_target, velocity_target]
      // send [position_measured, velocity_measured]
      tx_frame.id = (FUNC_TRANSMIT_PDO_2 << 7) | controller->device_id;
      tx_frame.size = 8;
      PositionController_setPositionTarget(&controller->position_controller, *((float *)rx_frame->data + 0));
      PositionController_setVelocityTarget(&controller->position_controller, *((float *)rx_frame->data + 1));
      *((float *)tx_frame.data + 0) = PositionController_getPositionMeasured(&controller->position_controller);
      *((float *)tx_frame.data + 1) = PositionController_getVelocityMeasured(&controller->position_controller);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      break;

    case FUNC_RECEIVE_PDO_3:
      // receive [position_target, torque_target]
      // send [position_measured, torque_measured]
      tx_frame.id = (FUNC_TRANSMIT_PDO_3 << 7) | controller->device_id;
      tx_frame.size = 8;
      PositionController_setPositionTarget(&controller->position_controller, *((float *)rx_frame->data + 0));
      PositionController_setTorqueTarget(&controller->position_controller, *((float *)rx_frame->data + 1));
      *((float *)tx_frame.data + 0) = PositionController_getPositionMeasured(&controller->position_controller);
      *((float *)tx_frame.data + 1) = PositionController_getTorqueMeasured(&controller->position_controller);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      break;

    case FUNC_RECEIVE_PDO_4:
      // fast-frame transmit-only
      break;

    case FUNC_FLASH:
      tx_frame.size = 1;
      if (*((uint8_t *)rx_frame->data)) {
        *((uint8_t *)tx_frame.data) = (uint8_t)MotorController_storeConfig(controller);
      }
      else {
        *((uint8_t *)tx_frame.data) = (uint8_t)MotorController_loadConfig(controller);
      }
      break;

    case FUNC_HEARTBEAT:
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      break;
  }

  if (tx_frame.size) {
    CAN_putTxFrame(&hfdcan1, &tx_frame);
  }
}

void MotorController_handleNMT(MotorController *controller, CAN_Frame *rx_frame) {
  if (!rx_frame->size) {
    controller->error |= ERROR_CAN_RX_FAULT;
    return;
  }

  uint8_t requested_state = *((uint8_t *)rx_frame->data);
  uint8_t addressed_node = *((uint8_t *)rx_frame->data + 1);

  if (addressed_node != controller->device_id) {
    return;
  }
  
  MotorController_setMode(controller, requested_state);
}

void MotorController_handleSDO(MotorController *controller, CAN_Frame *rx_frame, CAN_Frame *tx_frame) {
  if (!rx_frame->size) {
    controller->error |= ERROR_CAN_RX_FAULT;
    return;
  }

  uint8_t command = *((uint8_t *)rx_frame->data);

  // bits 5-7 in byte 1 is the client command specifier
  uint8_t ccs = command >> 5;

  // byte 1-2 is the OD index
  uint16_t parameter_id = *((uint16_t *)((uint8_t *)rx_frame->data + 1));

  if (parameter_id >= sizeof(MotorController)) {
    controller->error |= ERROR_CAN_RX_FAULT;
    return;
  }

  if (ccs == 1) {
    // download (write)
    *((uint32_t *)((uint8_t *)controller + parameter_id)) = *((uint32_t *)rx_frame->data + 1);

    // update the fast frame frequency
    if (controller->fast_frame_frequency)
      __HAL_TIM_SET_AUTORELOAD(&htim8, (10000 / controller->fast_frame_frequency) - 1);
    else {
      // by default running at 100Hz to reduce overhead
      __HAL_TIM_SET_AUTORELOAD(&htim8, (10000 / 100) - 1);
    }
  }
  else if (ccs == 2) {
    // upload (read)
    tx_frame->id = (FUNC_TRANSMIT_SDO << 7) | controller->device_id;
    tx_frame->size = 4;
    *((uint32_t *)tx_frame->data) = *((uint32_t *)((uint8_t *)controller + parameter_id));
  }
  else {
    controller->error |= ERROR_CAN_RX_FAULT;
  }
}

