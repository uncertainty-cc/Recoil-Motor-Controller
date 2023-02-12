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

  MotorController_loadConfig(controller);
  #if !LOAD_CONFIG_FROM_FLASH || !LOAD_CALIBRATION_FROM_FLASH
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
      // sleep
      break;

    case MODE_IDLE:
      __HAL_TIM_SET_AUTORELOAD(&htim3, 9999);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 2);
      break;

    case MODE_CALIBRATION:
      __HAL_TIM_SET_AUTORELOAD(&htim3, 1999);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 4);
      MotorController_reset(controller);
      PowerStage_enablePWM(&controller->powerstage);
      break;

    case MODE_DAMPING:
    case MODE_POSITION:
    case MODE_VELOCITY:
    case MODE_TORQUE:
    case MODE_CURRENT:
    case MODE_IQD_OVERRIDE:
    case MODE_VQD_OVERRIDE:
    case MODE_VALPHABETA_OVERRIDE:
    case MODE_VABC_OVERRIDE:
      __HAL_TIM_SET_AUTORELOAD(&htim3, 1999);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim3) / 2);
      if (controller->mode != MODE_IDLE) {
//        PowerStage_disable(&controller->powerstage);
        controller->mode = MODE_IDLE;
        SET_BITS(controller->error, ERROR_INVALID_MODE);
        return;  // return directly, do not update mode
      }
      MotorController_reset(controller);
      PowerStage_enablePWM(&controller->powerstage);
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
    controller->motor.flux_angle_offset                   = config->motor_flux_angle_offset;
  #endif
  #if LOAD_CONFIG_FROM_FLASH
    controller->device_id                                 = (uint8_t)config->device_id;
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
    controller->current_controller.i_kp                   = config->current_controller_i_kp;
    controller->current_controller.i_ki                   = config->current_controller_i_ki;
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

  return HAL_OK;
}

HAL_StatusTypeDef MotorController_storeConfig(MotorController *controller) {
  EEPROMConfig config;

  config.device_id                                      = (uint32_t)controller->device_id;
  config.firmware_version                               = controller->firmware_version;
  config.encoder_cpr                                    = controller->encoder.cpr;
  config.encoder_position_offset                        = controller->encoder.position_offset;
  config.encoder_filter_alpha                           = controller->encoder.filter_alpha;
  config.powerstage_undervoltage_threshold              = controller->powerstage.undervoltage_threshold;
  config.powerstage_overvoltage_threshold               = controller->powerstage.overvoltage_threshold;
  config.powerstage_bus_voltage_filter_alpha            = controller->powerstage.bus_voltage_filter_alpha;
  config.motor_pole_pairs                               = controller->motor.pole_pairs;
  config.motor_kv_rating                                = controller->motor.kv_rating;
  config.motor_phase_order                              = (int32_t)controller->motor.phase_order;
  config.motor_flux_angle_offset                        = controller->motor.flux_angle_offset;
  config.current_controller_i_kp                        = controller->current_controller.i_kp;
  config.current_controller_i_ki                        = controller->current_controller.i_ki;
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
  controller->position_controller.torque_measured = (8.3f * controller->current_controller.i_q_measured) / (float)controller->motor.kv_rating;

  PositionController_update(&controller->position_controller, controller->mode);

  if (controller->mode == MODE_POSITION
      || controller->mode == MODE_VELOCITY
      || controller->mode == MODE_TORQUE) {
    controller->current_controller.i_q_target = (controller->position_controller.torque_setpoint * (float)controller->motor.kv_rating) / 8.3f;
    controller->current_controller.i_d_target = 0.f;
  }


//  MotorController_updateSafety(controller);

  PowerStage_updateBusVoltage(&controller->powerstage);

  float theta = wrapTo2Pi((Encoder_getPositionMeasured(&controller->encoder) * (float)controller->motor.pole_pairs) - controller->motor.flux_angle_offset);
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
      || controller->mode == MODE_IQD_OVERRIDE
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

  while (controller->powerstage.bus_voltage_measured < 9) {
    {
      char str[128];
      sprintf(str, "waiting for bus voltage: %f\r\n", controller->powerstage.bus_voltage_measured);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    }
  }

  // open loop calibration
  float prev_v_alpha_target = controller->current_controller.v_alpha_setpoint;
  float prev_v_beta_target = controller->current_controller.v_beta_setpoint;

  float flux_angle_setpoint = 0;
  float voltage_setpoint = 0.2;

  MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);

  HAL_Delay(500);

  float phase_current = 0;

  while (phase_current < CALIBRATION_CURRENT) {
    HAL_Delay(100);
    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);

    voltage_setpoint += 0.1;
    phase_current = 1./3. * (fabs(controller->current_controller.i_a_measured) + fabs(controller->current_controller.i_b_measured) + fabs(controller->current_controller.i_c_measured));
    {
      char str[128];
      sprintf(str, "voltage: %f\tphase current: %f\r\n", voltage_setpoint, phase_current);
      HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    }

    if (voltage_setpoint > 12) {
      SET_BITS(controller->error, ERROR_CALIBRATION_ERROR);
      MotorController_setMode(controller, MODE_IDLE);
      return;
    }
  }

  HAL_Delay(1000);

  float start_position = Encoder_getPositionMeasured(&controller->encoder);

  // move one electrical revolution forward
  for (int16_t i=0; i<=500; i+=1) {
    flux_angle_setpoint = (i / 500.0f) * (2*M_PI);

    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);
  }
  HAL_Delay(500);

  float end_position = Encoder_getPositionMeasured(&controller->encoder);

  for (int16_t i=500; i>=0; i-=1) {
    flux_angle_setpoint = (i / 500.0f) * (2*M_PI);
    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);
  }

  flux_angle_setpoint = 0;
  MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
  HAL_Delay(500);

  start_position = 0.5 * Encoder_getPositionMeasured(&controller->encoder) + 0.5 * start_position;
  HAL_Delay(500);

  // release motor
  PowerStage_disablePWM(&controller->powerstage);

  controller->current_controller.v_alpha_setpoint = prev_v_alpha_target;
  controller->current_controller.v_beta_setpoint = prev_v_beta_target;

  float delta_position = end_position - start_position;

  {
    char str[128];
    sprintf(str, "initial encoder angle: %f\r\n", start_position);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    sprintf(str, "end encoder angle: %f\r\n", end_position);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    sprintf(str, "delta angle: %f\r\n", delta_position);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }


  if (fabsf(delta_position) < 0.1) {
    // motor did not rotate
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR: motor not rotating\r\n", strlen("ERROR: motor not rotating\r\n"), 10);
  }

  if (fabsf(fabsf(delta_position)*controller->motor.pole_pairs-(2*M_PI)) > 0.5f) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR: motor pole pair mismatch\r\n", strlen("ERROR: motor pole pair mismatch\r\n"), 10);
  }


  // set electrical angle
  controller->motor.flux_angle_offset = wrapTo2Pi(start_position * controller->motor.pole_pairs);

  {
    char str[128];
    sprintf(str, "offset angle: %f\r\n", controller->motor.flux_angle_offset);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }

  MotorController_storeConfig(controller);

  HAL_Delay(1000);

  MotorController_setMode(controller, MODE_IDLE);
}

void MotorController_handleCANMessage(MotorController *controller, CAN_Frame *rx_frame) {
  uint16_t device_id = (rx_frame->id) & 0b1111;
  if (device_id && device_id != controller->device_id) {
    return;
  }

  uint16_t func_id = (rx_frame->id) >> 4;

  CAN_Frame tx_frame;
  tx_frame.id = rx_frame->id;
  tx_frame.id_type = CAN_ID_STANDARD;
  tx_frame.frame_type = CAN_FRAME_DATA;
  tx_frame.size = 0;

  switch (func_id) {
    case CAN_ID_ESTOP:      // 0x00 []
      MotorController_setMode(controller, MODE_DISABLED);
      tx_frame.size = 2;
      *((uint16_t *)tx_frame.data) = 0xDEAD;
      break;

    case CAN_ID_ID:         // 0x01
      if (rx_frame->size) {
        // controller->device_id = *((uint8_t *)rx_frame->data);
        // TODO: restart logic
      }
      tx_frame.size = 1;
      *((uint8_t *)tx_frame.data) = controller->device_id;
      break;

    case CAN_ID_VERSION: // 0x02 [firmware_version]
      tx_frame.size = 4;
      *((uint32_t *)tx_frame.data) = controller->firmware_version;
      break;

    case CAN_ID_HEARTBEAT:  // 0x04 []
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      break;

    case CAN_ID_MODE_ERROR:  // 0x06 [mode, error] -> [mode, clear_error?]
      if (rx_frame->size) {
        MotorController_setMode(controller, *((uint16_t *)rx_frame->data));
        if (*((uint16_t *)rx_frame->data + 1)) {
          MotorController_clearError(controller);
        }
      }
      tx_frame.size = 4;
      *((uint16_t *)tx_frame.data) = (uint16_t)MotorController_getMode(controller);
      *((uint16_t *)tx_frame.data + 1) = (uint16_t)MotorController_getError(controller);
      break;

    case CAN_ID_FLASH:      // 0x0F [0/1]
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

    case CAN_ID_FAST_FRAME_0:   // 0x10 [position_target, torque_limit] -> [position_measured, torque_measured]
      controller->position_controller.position_target = *((float *)rx_frame->data);
      controller->position_controller.torque_limit = *((float *)rx_frame->data + 1);
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.position_measured;
      *((float *)tx_frame.data + 1) = controller->position_controller.torque_measured;
      break;

    case CAN_ID_FAST_FRAME_1:   // 0x11 [position_kp, position_ki]
      controller->position_controller.position_kp = *((float *)rx_frame->data);
      controller->position_controller.position_ki = *((float *)rx_frame->data + 1);
      break;

    case CAN_ID_ENCODER_CPR_OFFSET:  // 0x20 [cpr, position_offset]
      if (rx_frame->size) {
        controller->encoder.cpr = *((int32_t *)rx_frame->data);
        controller->encoder.position_offset = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((int32_t *)tx_frame.data) = controller->encoder.cpr;
      *((float *)tx_frame.data + 1) = controller->encoder.position_offset;
      break;

    case CAN_ID_ENCODER_FILTER:  // 0x21 [filter_alpha]
      if (rx_frame->size) {
        controller->encoder.filter_alpha = *((float *)rx_frame->data);
      }
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->encoder.filter_alpha;
      break;

    case CAN_ID_ENCODER_POSITION_RAW_N_ROTATIONS:  // 0x22 [position_raw, n_rotations]
      tx_frame.size = 8;
      *((int32_t *)tx_frame.data) = (int32_t)controller->encoder.position_raw;
      *((float *)tx_frame.data + 1) = controller->encoder.n_rotations;
      break;

    case CAN_ID_POWERSTAGE_VOLTAGE_THRESHOLD:  // 0x23 [undervoltage_threshold, overvoltage_threshold]
      if (rx_frame->size) {
        controller->powerstage.undervoltage_threshold = *((float *)rx_frame->data);
        controller->powerstage.overvoltage_threshold = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->powerstage.undervoltage_threshold;
      *((float *)tx_frame.data + 1) = controller->powerstage.overvoltage_threshold;
      break;

    case CAN_ID_POWERSTAGE_FILTER:  // 0x24 [bus_voltage_filter_alpha]
      if (rx_frame->size) {
        controller->powerstage.bus_voltage_filter_alpha = *((float *)rx_frame->data);
      }
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->powerstage.bus_voltage_filter_alpha;
      break;

    case CAN_ID_POWERSTAGE_BUS_VOLTAGE_MEASURED:  // 0x25 [bus_voltage_measured]
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->powerstage.bus_voltage_measured;
      break;

    case CAN_ID_MOTOR_POLE_PAIR_KV:  // 0x26 [pole_pairs, kv_rating]
      if (rx_frame->size) {
        controller->motor.pole_pairs = *((uint32_t *)rx_frame->data);
        controller->motor.kv_rating = *((uint32_t *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((uint32_t *)tx_frame.data) = controller->motor.pole_pairs;
      *((uint32_t *)tx_frame.data + 1) = controller->motor.kv_rating;
      break;

    case CAN_ID_MOTOR_PHASE_ORDER_FLUX_OFFSET:  // 0x27 [phase_order, flux_angle_offset]
      if (rx_frame->size) {
        controller->motor.phase_order = *((int8_t *)rx_frame->data);
        controller->motor.flux_angle_offset = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((int32_t *)tx_frame.data) = (int32_t)controller->motor.phase_order;
      *((float *)tx_frame.data + 1) = controller->motor.flux_angle_offset;
      break;

    case CAN_ID_CURRENT_KP_KI:  // 0x30 [i_kp, i_ki]
      if (rx_frame->size) {
        controller->current_controller.i_kp = *((float *)rx_frame->data);
        controller->current_controller.i_ki = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.i_kp;
      *((float *)tx_frame.data + 1) = controller->current_controller.i_ki;
      break;

    case CAN_ID_CURRENT_LIMIT:  // 0x31 [i_limit]
      if (rx_frame->size) {
        controller->current_controller.i_limit = *((float *)rx_frame->data);
      }
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->current_controller.i_limit;
      break;

    case CAN_ID_CURRENT_IA_IB_MEASURED:  // 0x32 [i_a_measured, i_b_measured]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.i_a_measured;
      *((float *)tx_frame.data + 1) = controller->current_controller.i_b_measured;
      break;

    case CAN_ID_CURRENT_IC_MEASURED:  // 0x33 [i_c_measured]
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->current_controller.i_c_measured;

    case CAN_ID_CURRENT_VA_VB_SETPOINT:  // 0x34 [v_a_setpoint, v_b_setpoint]
      if (rx_frame->size) {
        controller->current_controller.v_a_setpoint = *((float *)rx_frame->data);
        controller->current_controller.v_b_setpoint = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.v_a_setpoint;
      *((float *)tx_frame.data + 1) = controller->current_controller.v_b_setpoint;
      break;

    case CAN_ID_CURRENT_VC_SETPOINT:  // 0x35 [v_c_setpoint]
      if (rx_frame->size) {
        controller->current_controller.v_c_setpoint = *((float *)rx_frame->data);
      }
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->current_controller.v_c_setpoint;
      break;

    case CAN_ID_CURRENT_IALPHA_IBETA_MEASURED:  // 0x36 [i_alpha_measured, i_beta_measured]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.i_alpha_measured;
      *((float *)tx_frame.data + 1) = controller->current_controller.i_beta_measured;
      break;

    case CAN_ID_CURRENT_VALPHA_VBETA_SETPOINT:  // 0x37 [v_alpha_setpoint, v_beta_setpoint]
      if (rx_frame->size) {
        controller->current_controller.v_alpha_setpoint = *((float *)rx_frame->data);
        controller->current_controller.v_beta_setpoint = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.v_alpha_setpoint;
      *((float *)tx_frame.data + 1) = controller->current_controller.v_beta_setpoint;
      break;

    case CAN_ID_CURRENT_VQ_VD_TARGET:  // 0x38 [v_q_target, v_d_target]
      if (rx_frame->size) {
        controller->current_controller.v_q_target = *((float *)rx_frame->data);
        controller->current_controller.v_d_target = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.v_q_target;
      *((float *)tx_frame.data + 1) = controller->current_controller.v_d_target;
      break;

    case CAN_ID_CURRENT_VQ_VD_SETPOINT:  // 0x39 [v_q_setpoint,  v_d_setpoint]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.v_q_setpoint;
      *((float *)tx_frame.data + 1) = controller->current_controller.v_d_setpoint;
      break;

    case CAN_ID_CURRENT_IQ_ID_TARGET:  // 0x3A [i_q_target, i_d_target]
      if (rx_frame->size) {
        controller->current_controller.i_q_target = *((float *)rx_frame->data);
        controller->current_controller.i_d_target = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.i_q_target;
      *((float *)tx_frame.data + 1) = controller->current_controller.i_d_target;
      break;

    case CAN_ID_CURRENT_IQ_ID_MEASURED:  // 0x3B [i_q_measured, i_d_measured]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.i_q_measured;
      *((float *)tx_frame.data + 1) = controller->current_controller.i_d_measured;
      break;

    case CAN_ID_CURRENT_IQ_ID_SETPOINT:  // 0x3C [i_q_setpoint, i_d_setpoint]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.i_q_setpoint;
      *((float *)tx_frame.data + 1) = controller->current_controller.i_d_setpoint;
      break;

    case CAN_ID_CURRENT_IQ_ID_INTEGRATOR:  // 0x3D [i_q_integrator, i_d_integrator]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->current_controller.i_q_integrator;
      *((float *)tx_frame.data + 1) = controller->current_controller.i_d_integrator;
      break;

    case CAN_ID_POSITION_KP_KI:  // 0x40 [position_kp, position_ki]
      if (rx_frame->size) {
        controller->position_controller.position_kp = *((float *)rx_frame->data);
        controller->position_controller.position_ki = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.position_kp;
      *((float *)tx_frame.data + 1) = controller->position_controller.position_ki;
      break;

    case CAN_ID_VELOCITY_KP_KI:  // 0x41 [velocity_kp, velocity_ki]
      if (rx_frame->size) {
        controller->position_controller.velocity_kp = *((float *)rx_frame->data);
        controller->position_controller.velocity_ki = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.velocity_kp;
      *((float *)tx_frame.data + 1) = controller->position_controller.velocity_ki;
      break;

    case CAN_ID_TORQUE_VELOCITY_LIMIT:  // 0x42 [torque_limit, velocity_limit]
      if (rx_frame->size) {
        controller->position_controller.torque_limit = *((float *)rx_frame->data);
        controller->position_controller.velocity_limit = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.torque_limit;
      *((float *)tx_frame.data + 1) = controller->position_controller.velocity_limit;
      break;

    case CAN_ID_POSITION_LIMIT:  // 0x43 [position_limit_lower, position_limit_upper]
      if (rx_frame->size) {
        controller->position_controller.position_limit_lower = *((float *)rx_frame->data);
        controller->position_controller.position_limit_upper = *((float *)rx_frame->data + 1);
      }
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.position_limit_lower;
      *((float *)tx_frame.data + 1) = controller->position_controller.position_limit_upper;
      break;

    case CAN_ID_TORQUE_TARGET:  // 0x44 [torque_target]
      if (rx_frame->size) {
        controller->position_controller.torque_target = *((float *)rx_frame->data);
      }
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->position_controller.torque_target;
      break;

    case CAN_ID_TORQUE_MEASURED_SETPOINT:  // 0x45 [torque_measured, torque_setpoint]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.torque_measured;
      *((float *)tx_frame.data + 1) = controller->position_controller.torque_setpoint;
      break;

    case CAN_ID_VELOCITY_TARGET:  // 0x46 [velocity_target]
      if (rx_frame->size) {
        controller->position_controller.velocity_target = *((float *)rx_frame->data);
      }
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->position_controller.velocity_target;
      break;

    case CAN_ID_VELOCITY_MEASURED_SETPOINT:  // 0x47 [velocity_measured, velocity_setpoint]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.velocity_measured;
      *((float *)tx_frame.data + 1) = controller->position_controller.velocity_setpoint;
      break;

    case CAN_ID_POSITION_TARGET:  // 0x48 [position_target]
      if (rx_frame->size) {
        controller->position_controller.position_target = *((float *)rx_frame->data);
      }
      tx_frame.size = 4;
      *((float *)tx_frame.data) = controller->position_controller.position_target;
      break;

    case CAN_ID_POSITION_MEASURED_SETPOINT:  // 0x49 [position_measured, position_setpoint]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.position_measured;
      *((float *)tx_frame.data + 1) = controller->position_controller.position_setpoint;
      break;

    case CAN_ID_POSITION_VELOCITY_INTEGRATOR:  // 0x49 [position_integrator, velocity_integrator]
      tx_frame.size = 8;
      *((float *)tx_frame.data) = controller->position_controller.position_integrator;
      *((float *)tx_frame.data + 1) = controller->position_controller.velocity_integrator;
      break;

    case CAN_ID_PING:  // 0x7F
      tx_frame.size = 1;
      *((uint8_t *)tx_frame.data) = controller->device_id;
      break;
  }

  if (tx_frame.size) {
    CAN_putTxFrame(&hfdcan1, &tx_frame);
  }
}

