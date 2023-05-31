/*
 * motor_controller.c
 *
 *  Created on: Aug 25, 2022
 *      Author: TK
 */

#include "motor_controller.h"

#define FLASH_CONFIG_ADDRESS    0x0801F800U  // Bank 1, Page 63
#define FLASH_CONFIG_BANK       FLASH_BANK_1
#define FLASH_CONFIG_PAGE       63
#define FLASH_CONFIG_SIZE       32


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
extern UART_HandleTypeDef huart2;

void MotorController_init(MotorController *controller) {
  controller->mode = MODE_DISABLED;
  controller->device_id = DEVICE_CAN_ID;
  controller->firmware_version = FIRMWARE_VERSION;

  HAL_StatusTypeDef status = HAL_OK;

//  status |= CAN_init(&hfdcan1, controller->device_id, 0b1111);
  status |= CAN_init(&hfdcan1, 0, 0);

  status |= Encoder_init(&controller->encoder, &hi2c1);
  status |= PowerStage_init(&controller->powerstage, &htim1, &hadc1, &hadc2);
  status |= Motor_init(&controller->motor);

  status |= CurrentController_init(&controller->current_controller);
  status |= PositionController_init(&controller->position_controller);

  status |= MotorController_loadConfig(controller);
  #if !LOAD_ID_FROM_FLASH || !LOAD_CONFIG_FROM_FLASH || !LOAD_CALIBRATION_FROM_FLASH
    MotorController_storeConfig(controller);
  #endif

  MotorController_reset(controller);

  status |= HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);       // LED PWM timer

  __HAL_TIM_SET_AUTORELOAD(&htim3, 9999);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  status |= HAL_OPAMP_Start(&hopamp1);
  status |= HAL_OPAMP_Start(&hopamp2);
  status |= HAL_OPAMP_Start(&hopamp3);

  status |= HAL_ADCEx_InjectedStart(&hadc1);
  status |= HAL_ADCEx_InjectedStart(&hadc2);

  status |= HAL_TIM_Base_Start_IT(&htim2);    // safety watchdog timer
  status |= HAL_TIM_Base_Start(&htim6);       // time keeper timer

  PowerStage_start(&controller->powerstage);

  if (status != HAL_OK) {
    SET_BITS(controller->error, ERROR_INITIALIZATION_ERROR);
    MotorController_setMode(controller, MODE_IDLE);

    __HAL_TIM_SET_AUTORELOAD(&htim3, 999);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 2);
    while (1) {
      // error loop
    }
  }

  HAL_Delay(100);
  PowerStage_calibratePhaseCurrentOffset(&controller->powerstage);

  // change mode to idle
  MotorController_clearError(controller);
  MotorController_setMode(controller, MODE_IDLE);
}

void MotorController_reset(MotorController *controller) {
  // clear all intermediate states
  controller->position_controller.position_setpoint = controller->position_controller.position_measured;
  controller->position_controller.position_integrator = 0.f;
  controller->position_controller.velocity_setpoint = controller->position_controller.velocity_measured;
  controller->position_controller.velocity_integrator = 0.f;

  controller->current_controller.i_q_integrator = 0.f;
  controller->current_controller.i_d_integrator = 0.f;
  controller->current_controller.v_q_setpoint = 0.f;
  controller->current_controller.v_d_setpoint = 0.f;
  controller->current_controller.v_alpha_setpoint = 0.f;
  controller->current_controller.v_beta_setpoint = 0.f;
  controller->current_controller.v_a_setpoint = 0.f;
  controller->current_controller.v_b_setpoint = 0.f;
  controller->current_controller.v_c_setpoint = 0.f;

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
      PowerStage_disablePWM(&controller->powerstage);
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
  EEPROMConfig *config = (EEPROMConfig *)FLASH_CONFIG_ADDRESS;
  #if LOAD_CALIBRATION_FROM_FLASH
    if (isnan(config->encoder_flux_offset)) {
      return HAL_ERROR;
    }
    controller->encoder.flux_offset                       = config->encoder_flux_offset;
  #endif
  #if LOAD_ID_FROM_FLASH
    controller->device_id                                 = (uint8_t)config->device_id;
  #endif
  #if LOAD_CONFIG_FROM_FLASH
    controller->firmware_version                          = config->firmware_version;
    controller->encoder.cpr                               = config->encoder_cpr;
    controller->encoder.position_offset                   = config->encoder_position_offset;
    controller->encoder.filter_alpha                      = config->encoder_filter_alpha;
    controller->powerstage.undervoltage_threshold         = config->powerstage_undervoltage_threshold;
    controller->powerstage.overvoltage_threshold          = config->powerstage_overvoltage_threshold;
    controller->powerstage.bus_voltage_filter_alpha       = config->powerstage_bus_voltage_filter_alpha;
    controller->motor.pole_pairs                          = config->motor_pole_pairs;
    controller->motor.kv_rating                           = config->motor_kv_rating;
    controller->motor.phase_order                         = (int8_t)config->motor_phase_order;
    controller->motor.phase_resistance                    = config->motor_phase_resistance;
    controller->motor.phase_inductance                    = config->motor_phase_inductance;
    controller->current_controller.i_bandwidth            = config->current_controller_i_bandwidth;
    controller->current_controller.i_limit                = config->current_controller_i_limit;
    controller->position_controller.position_kp           = config->position_controller_position_kp;
    controller->position_controller.position_ki           = config->position_controller_position_ki;
    controller->position_controller.velocity_kp           = config->position_controller_velocity_kp;
    controller->position_controller.velocity_ki           = config->position_controller_velocity_ki;
    controller->position_controller.torque_limit          = config->position_controller_torque_limit;
    controller->position_controller.velocity_limit        = config->position_controller_velocity_limit;
    controller->position_controller.position_limit_upper  = config->position_controller_position_limit_upper;
    controller->position_controller.position_limit_lower  = config->position_controller_position_limit_lower;
  #endif

  CurrentController_setPIGain(&controller->current_controller,
      controller->motor.phase_resistance,
      controller->motor.phase_inductance);

  return HAL_OK;
}

HAL_StatusTypeDef MotorController_storeConfig(MotorController *controller) {
  EEPROMConfig config;

  config.device_id                                      = (uint32_t)controller->device_id;
  config.firmware_version                               = controller->firmware_version;
  config.encoder_cpr                                    = controller->encoder.cpr;
  config.encoder_position_offset                        = controller->encoder.position_offset;
  config.encoder_filter_alpha                           = controller->encoder.filter_alpha;
  config.encoder_flux_offset                            = controller->encoder.flux_offset;
  config.powerstage_undervoltage_threshold              = controller->powerstage.undervoltage_threshold;
  config.powerstage_overvoltage_threshold               = controller->powerstage.overvoltage_threshold;
  config.powerstage_bus_voltage_filter_alpha            = controller->powerstage.bus_voltage_filter_alpha;
  config.motor_pole_pairs                               = controller->motor.pole_pairs;
  config.motor_kv_rating                                = controller->motor.kv_rating;
  config.motor_phase_order                              = (int32_t)controller->motor.phase_order;
  config.motor_phase_resistance                         = controller->motor.phase_resistance;
  config.motor_phase_inductance                         = controller->motor.phase_inductance;
  config.current_controller_i_bandwidth                 = controller->current_controller.i_bandwidth;
  config.current_controller_i_limit                     = controller->current_controller.i_limit;
  config.position_controller_position_kp                = controller->position_controller.position_kp;
  config.position_controller_position_ki                = controller->position_controller.position_ki;
  config.position_controller_velocity_kp                = controller->position_controller.velocity_kp;
  config.position_controller_velocity_ki                = controller->position_controller.velocity_ki;
  config.position_controller_torque_limit               = controller->position_controller.torque_limit;
  config.position_controller_velocity_limit             = controller->position_controller.velocity_limit;
  config.position_controller_position_limit_upper       = controller->position_controller.position_limit_upper;
  config.position_controller_position_limit_lower       = controller->position_controller.position_limit_lower;

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
//    uint32_t error = HAL_FLASH_GetError();
    HAL_FLASH_Lock();
    return HAL_ERROR;
  }

  /* Program the user Flash area word by word*/
  for (uint16_t i=0; i<FLASH_CONFIG_SIZE; i+=1) {
    uint64_t buf = (uint64_t)*(((uint64_t *)(&config)) + i);

    uint32_t target_address = FLASH_CONFIG_ADDRESS + i*8;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, target_address, buf) != HAL_OK) {
//      uint32_t error = HAL_FLASH_GetError();
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
  // CPU time: 74%, 37 us total
  // 20kHz refresh rate requirements:
  //  - -O2 optimization
  //  - 50% I2C transaction
  //  -

  // this is the most time-sensitive
  // takes 1.42 us to run (2.8%) under -O2
  PowerStage_updatePhaseCurrent(&controller->powerstage,
      &controller->current_controller.i_a_measured,
      &controller->current_controller.i_b_measured,
      &controller->current_controller.i_c_measured,
      controller->motor.phase_order);

  // also quite time-sensitive
  // takes 10.92 us to run (22%) under -O2
  Encoder_update(&controller->encoder, 1.f / (20000.f));

  controller->position_controller.position_measured = Encoder_getPosition(&controller->encoder);
  controller->position_controller.velocity_measured = Encoder_getVelocity(&controller->encoder);
  // 1.75 is a magic number.... need to find out why it's different from the theoretical value
  controller->position_controller.torque_measured = (1.75* 8.3f * controller->current_controller.i_q_measured) / (float)controller->motor.kv_rating;

  PositionController_update(&controller->position_controller, controller->mode);

  if (controller->mode == MODE_POSITION
      || controller->mode == MODE_VELOCITY
      || controller->mode == MODE_TORQUE) {
    // same here, the 1.75 magic number...
    controller->current_controller.i_q_target = (controller->position_controller.torque_setpoint * (float)controller->motor.kv_rating) / (1.75* 8.3f);
    controller->current_controller.i_d_target = 0.f;
  }
  else {
    // MODE_CURRENT
    /*
     * user sets `controller->i_q_target` and `controller->i_d_target`
     */
  }


//  MotorController_updateSafety(controller);

  PowerStage_updateBusVoltage(&controller->powerstage);

  float theta = wrapTo2Pi(
      (Encoder_getPositionMeasured(&controller->encoder) * (float)controller->motor.pole_pairs)
      - controller->encoder.flux_offset
      );
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);

  CurrentController_update(&controller->current_controller,
      controller->mode,
      sin_theta,
      cos_theta,
      controller->powerstage.bus_voltage_measured);

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
  float error_table[ENCODER_LUT_RESOLUTION * 32];

  // store normal running v_alpha and v_beta values. We need to change this during the calibration.
  float prev_v_alpha_target = controller->current_controller.v_alpha_setpoint;
  float prev_v_beta_target = controller->current_controller.v_beta_setpoint;

  // starting voltage setpoint (V)
  float voltage_setpoint = 0.2;

  float flux_angle_setpoint = 0;

  MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);

  HAL_Delay(500);

  float phase_current = 0;

  // gradually ramp up the voltage setpoint until we reach target phase current value.
  while (phase_current < CALIBRATION_CURRENT) {
    HAL_Delay(100);
    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);

    voltage_setpoint += 0.1f;
    phase_current = 1.f/3.f * (fabs(controller->current_controller.i_a_measured) + fabs(controller->current_controller.i_b_measured) + fabs(controller->current_controller.i_c_measured));
    {
      char str[128];
      sprintf(str, "voltage: %f\tphase current: %f\r\n", voltage_setpoint, phase_current);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    }

    // if we cannot even reach the target phase current at 12V, there's something wrong. End calibration.
    if (voltage_setpoint > 12.f) {
      SET_BITS(controller->error, ERROR_CALIBRATION_ERROR);
      MotorController_setMode(controller, MODE_IDLE);
      return;
    }
  }

  HAL_Delay(1000);


  // move one mechanical revolution forward
  for (uint32_t i=0; i<128 * 14; i+=1) {
    flux_angle_setpoint = ((float)i / (128.f*14.f)) * (2*M_PI) * controller->motor.pole_pairs;

    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);

    float error = Encoder_getPositionMeasured(&controller->encoder) * controller->motor.pole_pairs - flux_angle_setpoint;
    error_table[i] = error;

//    {
//      char str[128];
//      sprintf(str, "%d: %f\r\n", i, error_table[i]);
//      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
//    }
  }

  HAL_Delay(500);

  // move one mechanical revolution backward
  for (uint32_t i=128 * 14; i>0; i-=1) {
    flux_angle_setpoint = ((float)i / (128.f*14.f)) * (2*M_PI) * controller->motor.pole_pairs;

    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);

    float error = Encoder_getPositionMeasured(&controller->encoder) * controller->motor.pole_pairs - flux_angle_setpoint;
    error_table[i-1] = 0.5f * (error_table[i-1] + error);

//    {
//      char str[128];
//      sprintf(str, "%d: %f\r\n", i-1, error_table[i-1]);
//      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
//    }
  }

  // release motor
  PowerStage_disablePWM(&controller->powerstage);


  // Calculate average offset
  float flux_offset_sum = 0;
  for (uint32_t i=0; i<128 * 14; i+=1) {
    flux_offset_sum += error_table[i];
  }
//  controller->encoder.flux_offset = wrapTo2Pi(flux_offset_sum / (float)(N_LUT * controller->motor.pole_pairs));
  controller->encoder.flux_offset = flux_offset_sum / (float)(ENCODER_LUT_RESOLUTION * controller->motor.pole_pairs);

  // should be 6.178778

  {
    char str[128];
    sprintf(str, "offset angle: %f %f\r\n", flux_offset_sum / (float)(ENCODER_LUT_RESOLUTION * controller->motor.pole_pairs), controller->encoder.flux_offset);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }

// Moving average to filter out cogging ripple
  int16_t window = ENCODER_LUT_RESOLUTION;
  int16_t lut_offset = ((controller->motor.pole_pairs*M_2PI_F)-error_table[0])*ENCODER_LUT_RESOLUTION / (controller->motor.pole_pairs*M_2PI_F);

  // make sure lut_offset is always >= 0
  if (lut_offset < 0) {
    lut_offset += ENCODER_LUT_RESOLUTION;
  }

  {
    char str[128];
    sprintf(str, "lut_offset: %d\r\n", lut_offset);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }

  for (int16_t i=0; i<ENCODER_LUT_RESOLUTION; i+=1) {
    float moving_avg = 0;

    for (int16_t j=-window/2; j<window/2; j+=1) {
      int32_t index = (i * controller->motor.pole_pairs) + j;
      // make sure index is always >= 0
      if (index < 0) {
        index += controller->motor.pole_pairs * ENCODER_LUT_RESOLUTION;
      }
      // make sure index is always < controller->motor.pole_pairs * ENCODER_LUT_RESOLUTION
      else if (index >= controller->motor.pole_pairs * ENCODER_LUT_RESOLUTION) {
        index -= controller->motor.pole_pairs * ENCODER_LUT_RESOLUTION;
      }
      moving_avg += error_table[index];
    }

    moving_avg = moving_avg / window;
    int32_t lut_index = lut_offset + i;
//    if (lut_index >= N_LUT) {
//      lut_index -= N_LUT;
//    }
    lut_index = lut_index % ENCODER_LUT_RESOLUTION;
    controller->encoder.flux_offset_table[lut_index] = moving_avg - controller->encoder.flux_offset;

    {
      char str[128];
      sprintf(str, "lut_index: %d, %f\r\n", lut_index, moving_avg - controller->encoder.flux_offset);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    }
  }


  CurrentController_setPIGain(&controller->current_controller,
      controller->motor.phase_resistance,
      controller->motor.phase_inductance);

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
  uint16_t device_id = (rx_frame->id) & 0b111111;
  if (device_id && device_id != controller->device_id) {
    return;
  }

  uint16_t func_id = (rx_frame->id) >> 6;

  CAN_Frame tx_frame;
  tx_frame.id = rx_frame->id;
  tx_frame.id_type = CAN_ID_STANDARD;
  tx_frame.frame_type = CAN_FRAME_DATA;
  tx_frame.size = 0;

  switch (func_id) {
    case CAN_ID_ESTOP:            // 0x00
      MotorController_setMode(controller, MODE_DISABLED);
      tx_frame.size = 2;
      *((uint16_t *)tx_frame.data) = 0xDEAD;
      break;

    case CAN_ID_INFO:             // 0x01
      if (rx_frame->size) {
        controller->device_id = *((uint8_t *)rx_frame->data);
      }
      tx_frame.size = 8;
      *((uint8_t *)(tx_frame.data)) = controller->device_id;
      *((uint32_t *)(tx_frame.data + 4)) = controller->firmware_version;
      break;

    case CAN_ID_SAFETY_WATCHDOG:  // 0x02
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      break;

    case CAN_ID_MODE:             // 0x05 [mode, error] -> [mode, clear_error?]
      if (rx_frame->size) {
        MotorController_setMode(controller, *((uint16_t *)rx_frame->data));
        if (*((uint16_t *)(rx_frame->data + 2))) {
          MotorController_clearError(controller);
        }
      }
      tx_frame.size = 4;
      *((uint16_t *)tx_frame.data) = (uint16_t)MotorController_getMode(controller);
      *((uint16_t *)(tx_frame.data + 2)) = (uint16_t)MotorController_getError(controller);
      break;

    case CAN_ID_FLASH:            // 0x0E [0/1]
      if (rx_frame->size) {
        tx_frame.size = 1;
        if (*((uint8_t *)rx_frame->data)) {
          *((uint8_t *)tx_frame.data) = (uint8_t)MotorController_storeConfig(controller);
        }
        else {
          *((uint8_t *)tx_frame.data) = (uint8_t)MotorController_loadConfig(controller);
        }
      }
      break;

    case CAN_ID_USR_PARAM_READ:   // 0x10
      MotorController_handleCANRead(controller, *((uint8_t *)rx_frame->data), &tx_frame);
      break;

    case CAN_ID_USR_PARAM_WRITE:  // 0x11
      MotorController_handleCANWrite(controller, *((uint8_t *)rx_frame->data), (uint8_t *)rx_frame->data + 4);
      break;

    case CAN_ID_USR_FAST_FRAME_0: // 0x12 [position_kp, position_ki]
      controller->position_controller.position_target = *((float *)rx_frame->data);
      controller->position_controller.torque_target = *((float *)rx_frame->data + 1);
      *((float *)tx_frame.data) = controller->position_controller.position_measured;
      *((float *)(tx_frame.data + 4)) = controller->position_controller.torque_measured;
      break;

    case CAN_ID_USR_FAST_FRAME_1: // 0x13 [position_kp, position_ki]
      controller->position_controller.position_kp = *((float *)rx_frame->data);
      controller->position_controller.position_ki = *((float *)(rx_frame->data + 4));
      break;

    case CAN_ID_PING:             // 0x1F
      tx_frame.size = 1;
      *((uint8_t *)tx_frame.data) = controller->device_id;
      break;
  }

  if (tx_frame.size) {
    CAN_putTxFrame(&hfdcan1, &tx_frame);
  }
}

void MotorController_handleCANRead(MotorController *controller, Command command, CAN_Frame *tx_frame) {
  tx_frame->size = 8;
  *((uint8_t *)tx_frame->data) = command;
  switch (command) {
    case CMD_ENCODER_CPR:
      *((int32_t *)(tx_frame->data + 4)) = controller->encoder.cpr;
      break;
    case CMD_ENCODER_OFFSET:
      *((float *)(tx_frame->data + 4)) = controller->encoder.position_offset;
      break;
    case CMD_ENCODER_FILTER:
      *((float *)(tx_frame->data + 4)) = controller->encoder.filter_alpha;
      break;
    case CMD_ENCODER_FLUX_OFFSET:
      *((float *)(tx_frame->data + 4)) = controller->encoder.flux_offset;
      break;
    case CMD_ENCODER_POSITION_RAW:
      *((int32_t *)(tx_frame->data + 4)) = (int32_t)controller->encoder.position_raw;
      break;
    case CMD_ENCODER_N_ROTATIONS:
      *((int32_t *)(tx_frame->data + 4)) = controller->encoder.n_rotations;
      break;
    case CMD_POWERSTAGE_VOLTAGE_THRESHOLD_LOW:
      *((float *)(tx_frame->data + 4)) = controller->powerstage.undervoltage_threshold;
      break;
    case CMD_POWERSTAGE_VOLTAGE_THRESHOLD_HIGH:
      *((float *)(tx_frame->data + 4)) = controller->powerstage.overvoltage_threshold;
      break;
    case CMD_POWERSTAGE_FILTER:
      *((float *)(tx_frame->data + 4)) = controller->powerstage.bus_voltage_filter_alpha;
      break;
    case CMD_POWERSTAGE_BUS_VOLTAGE_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->powerstage.bus_voltage_measured;
      break;
    case CMD_MOTOR_POLE_PAIR:
      *((uint32_t *)(tx_frame->data + 4)) = controller->motor.pole_pairs;
      break;
    case CMD_MOTOR_KV:
      *((uint32_t *)(tx_frame->data + 4)) = controller->motor.kv_rating;
      break;
    case CMD_MOTOR_PHASE_ORDER:
      *((int32_t *)(tx_frame->data + 4)) = (int32_t)controller->motor.phase_order;
      break;
    case CMD_MOTOR_PHASE_RESISTANCE:
      *((float *)(tx_frame->data + 4)) = controller->motor.phase_resistance;
      break;
    case CMD_MOTOR_PHASE_INDUCTANCE:
      *((float *)(tx_frame->data + 4)) = controller->motor.phase_inductance;
      break;
    case CMD_CURRENT_KP:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_kp;
      break;
    case CMD_CURRENT_KI:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_ki;
      break;
    case CMD_CURRENT_BANDWIDTH:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_bandwidth;
      break;
    case CMD_CURRENT_LIMIT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_limit;
      break;
    case CMD_CURRENT_IA_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_a_measured;
      break;
    case CMD_CURRENT_IB_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_b_measured;
      break;
    case CMD_CURRENT_IC_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_c_measured;
      break;
    case CMD_CURRENT_VA_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_a_setpoint;
      break;
    case CMD_CURRENT_VB_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_b_setpoint;
      break;
    case CMD_CURRENT_VC_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_c_setpoint;
      break;
    case CMD_CURRENT_IALPHA_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_alpha_measured;
      break;
    case CMD_CURRENT_IBETA_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_beta_measured;
      break;
    case CMD_CURRENT_VALPHA_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_alpha_setpoint;
      break;
    case CMD_CURRENT_VBETA_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_beta_setpoint;
      break;
    case CMD_CURRENT_VQ_TARGET:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_q_target;
      break;
    case CMD_CURRENT_VD_TARGET:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_d_target;
      break;
    case CMD_CURRENT_VQ_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_q_setpoint;
      break;
    case CMD_CURRENT_VD_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.v_d_setpoint;
      break;
    case CMD_CURRENT_IQ_TARGET:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_q_target;
      break;
    case CMD_CURRENT_ID_TARGET:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_d_target;
      break;
    case CMD_CURRENT_IQ_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_q_measured;
      break;
    case CMD_CURRENT_ID_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_d_measured;
      break;
    case CMD_CURRENT_IQ_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_q_setpoint;
      break;
    case CMD_CURRENT_ID_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_d_setpoint;
      break;
    case CMD_CURRENT_IQ_INTEGRATOR:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_q_integrator;
      break;
    case CMD_CURRENT_ID_INTEGRATOR:
      *((float *)(tx_frame->data + 4)) = controller->current_controller.i_d_integrator;
      break;
    case CMD_POSITION_KP:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.position_kp;
      break;
    case CMD_POSITION_KI:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.position_ki;
      break;
    case CMD_VELOCITY_KP:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.velocity_kp;
      break;
    case CMD_VELOCITY_KI:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.velocity_ki;
      break;
    case CMD_TORQUE_LIMIT:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.torque_limit;
      break;
    case CMD_VELOCITY_LIMIT:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.velocity_limit;
      break;
    case CMD_POSITION_LIMIT_LOW:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.position_limit_lower;
      break;
    case CMD_POSITION_LIMIT_HIGH:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.position_limit_upper;
      break;
    case CMD_TORQUE_TARGET:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.torque_target;
      break;
    case CMD_TORQUE_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.torque_measured;
      break;
    case CMD_TORQUE_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.torque_setpoint;
      break;
    case CMD_VELOCITY_TARGET:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.velocity_target;
      break;
    case CMD_VELOCITY_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.velocity_measured;
      break;
    case CMD_VELOCITY_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.velocity_setpoint;
      break;
    case CMD_POSITION_TARGET:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.position_target;
      break;
    case CMD_POSITION_MEASURED:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.position_measured;
      break;
    case CMD_POSITION_SETPOINT:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.position_setpoint;
      break;
    case CMD_VELOCITY_INTEGRATOR:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.velocity_integrator;
      break;
    case CMD_POSITION_INTEGRATOR:
      *((float *)(tx_frame->data + 4)) = controller->position_controller.position_integrator;
      break;
    default:
      break;
  }
}

void MotorController_handleCANWrite(MotorController *controller, Command command, uint8_t *rx_data) {
  switch (command) {
    case CMD_ENCODER_CPR:
      controller->encoder.cpr = *((int32_t *)rx_data);
      break;
    case CMD_ENCODER_OFFSET:
      controller->encoder.position_offset = *((float *)rx_data);
      break;
    case CMD_ENCODER_FILTER:
      controller->encoder.filter_alpha = *((float *)rx_data);
      break;
    case CMD_ENCODER_FLUX_OFFSET:
      controller->encoder.flux_offset = *((float *)rx_data);
      break;
    case CMD_ENCODER_POSITION_RAW:
      controller->encoder.position_raw = (int16_t)*((int32_t *)rx_data);
      break;
    case CMD_ENCODER_N_ROTATIONS:
      controller->encoder.n_rotations = *((int32_t *)rx_data);
      break;
    case CMD_POWERSTAGE_VOLTAGE_THRESHOLD_LOW:
      controller->powerstage.undervoltage_threshold = *((float *)rx_data);
      break;
    case CMD_POWERSTAGE_VOLTAGE_THRESHOLD_HIGH:
      controller->powerstage.overvoltage_threshold = *((float *)rx_data);
      break;
    case CMD_POWERSTAGE_FILTER:
      controller->powerstage.bus_voltage_filter_alpha = *((float *)rx_data);
      break;
    case CMD_POWERSTAGE_BUS_VOLTAGE_MEASURED:
//      controller->powerstage.bus_voltage_measured = *((float *)rx_data);
      break;
    case CMD_MOTOR_POLE_PAIR:
      controller->motor.pole_pairs = *((uint32_t *)rx_data);
      break;
    case CMD_MOTOR_KV:
      controller->motor.kv_rating = *((uint32_t *)rx_data);
      break;
    case CMD_MOTOR_PHASE_ORDER:
      controller->motor.phase_order = (int8_t)*((int32_t *)rx_data);
      break;
    case CMD_MOTOR_PHASE_RESISTANCE:
      controller->motor.phase_resistance = *((float *)rx_data);
      break;
    case CMD_MOTOR_PHASE_INDUCTANCE:
      controller->motor.phase_inductance = *((float *)rx_data);
      break;
    case CMD_CURRENT_KP:
      controller->current_controller.i_kp = *((float *)rx_data);
      break;
    case CMD_CURRENT_KI:
      controller->current_controller.i_ki = *((float *)rx_data);
      break;
    case CMD_CURRENT_BANDWIDTH:
      controller->current_controller.i_bandwidth = *((float *)rx_data);
      break;
    case CMD_CURRENT_LIMIT:
      controller->current_controller.i_limit = *((float *)rx_data);
      break;
    case CMD_CURRENT_IA_MEASURED:
      // this is a read-only term
      break;
    case CMD_CURRENT_IB_MEASURED:
      // this is a read-only term
      break;
    case CMD_CURRENT_IC_MEASURED:
      // this is a read-only term
      break;
    case CMD_CURRENT_VA_SETPOINT:
      controller->current_controller.v_a_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_VB_SETPOINT:
      controller->current_controller.v_b_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_VC_SETPOINT:
      controller->current_controller.v_c_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_IALPHA_MEASURED:
      // this is a read-only term
      break;
    case CMD_CURRENT_IBETA_MEASURED:
      // this is a read-only term
      break;
    case CMD_CURRENT_VALPHA_SETPOINT:
      controller->current_controller.v_alpha_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_VBETA_SETPOINT:
      controller->current_controller.v_beta_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_VQ_TARGET:
      controller->current_controller.v_q_target = *((float *)rx_data);
      break;
    case CMD_CURRENT_VD_TARGET:
      controller->current_controller.v_d_target = *((float *)rx_data);
      break;
    case CMD_CURRENT_VQ_SETPOINT:
      controller->current_controller.v_q_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_VD_SETPOINT:
      controller->current_controller.v_d_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_IQ_TARGET:
      controller->current_controller.i_q_target = *((float *)rx_data);
      break;
    case CMD_CURRENT_ID_TARGET:
      controller->current_controller.i_d_target = *((float *)rx_data);
      break;
    case CMD_CURRENT_IQ_MEASURED:
      // this is a read-only term
      break;
    case CMD_CURRENT_ID_MEASURED:
      // this is a read-only term
      break;
    case CMD_CURRENT_IQ_SETPOINT:
      controller->current_controller.i_q_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_ID_SETPOINT:
      controller->current_controller.i_d_setpoint = *((float *)rx_data);
      break;
    case CMD_CURRENT_IQ_INTEGRATOR:
      controller->current_controller.i_q_integrator = *((float *)rx_data);
      break;
    case CMD_CURRENT_ID_INTEGRATOR:
      controller->current_controller.i_d_integrator = *((float *)rx_data);
      break;
    case CMD_POSITION_KP:
      controller->position_controller.position_kp = *((float *)rx_data);
      break;
    case CMD_POSITION_KI:
      controller->position_controller.position_ki = *((float *)rx_data);
      break;
    case CMD_VELOCITY_KP:
      controller->position_controller.velocity_kp = *((float *)rx_data);
      break;
    case CMD_VELOCITY_KI:
      controller->position_controller.velocity_ki = *((float *)rx_data);
      break;
    case CMD_TORQUE_LIMIT:
      controller->position_controller.torque_limit = *((float *)rx_data);
      break;
    case CMD_VELOCITY_LIMIT:
      controller->position_controller.velocity_limit = *((float *)rx_data);
      break;
    case CMD_POSITION_LIMIT_LOW:
      controller->position_controller.position_limit_lower = *((float *)rx_data);
      break;
    case CMD_POSITION_LIMIT_HIGH:
      controller->position_controller.position_limit_upper = *((float *)rx_data);
      break;
    case CMD_TORQUE_TARGET:
      controller->position_controller.torque_target = *((float *)rx_data);
      break;
    case CMD_TORQUE_MEASURED:
      // this is a read-only term
      break;
    case CMD_TORQUE_SETPOINT:
      controller->position_controller.torque_setpoint = *((float *)rx_data);
      break;
    case CMD_VELOCITY_TARGET:
      controller->position_controller.velocity_target = *((float *)rx_data);
      break;
    case CMD_VELOCITY_MEASURED:
      // this is a read-only term
      break;
    case CMD_VELOCITY_SETPOINT:
      controller->position_controller.velocity_setpoint = *((float *)rx_data);
      break;
    case CMD_POSITION_TARGET:
      controller->position_controller.position_target = *((float *)rx_data);
      break;
    case CMD_POSITION_MEASURED:
      // this is a read-only term
      break;
    case CMD_POSITION_SETPOINT:
      controller->position_controller.position_setpoint = *((float *)rx_data);
      break;
    case CMD_VELOCITY_INTEGRATOR:
      controller->position_controller.velocity_integrator = *((float *)rx_data);
      break;
    case CMD_POSITION_INTEGRATOR:
      controller->position_controller.position_integrator = *((float *)rx_data);
      break;
    default:
      break;
  }
}

