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
#define FLASH_CONFIG_SIZE       12

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart3;

void MotorController_init(MotorController *controller) {
  controller->mode = MODE_DISABLED;
  controller->device_id = DEVICE_CAN_ID;
  controller->firmware_version = FIRMWARE_VERSION;

  FDCAN_FilterTypeDef filter_config;
  filter_config.IdType = FDCAN_STANDARD_ID;
  filter_config.FilterIndex = 0;
  filter_config.FilterType = FDCAN_FILTER_MASK;
  filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter_config.FilterID1 = controller->device_id;    // filter
  filter_config.FilterID2 = 0;//0b1111;                   // mask

  HAL_StatusTypeDef status = HAL_OK;

  status |= HAL_FDCAN_ConfigFilter(&hfdcan1, &filter_config);

  status |= HAL_FDCAN_Start(&hfdcan1);

  status |= HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  Encoder_init(&controller->encoder, &hspi1, &htim6);
  PowerStage_init(&controller->powerstage, &htim1, &hadc1, &hadc2);
  Motor_init(&controller->motor);

  CurrentController_init(&controller->current_controller);
  PositionController_init(&controller->position_controller);

#if OVERWRITE_CONFIG
  MotorController_storeConfig(controller);
#else
  MotorController_loadConfig(controller);
#endif

  PowerStage_start(&controller->powerstage);

  status |= HAL_OPAMP_Start(&hopamp1);
  status |= HAL_OPAMP_Start(&hopamp2);
  status |= HAL_OPAMP_Start(&hopamp3);

  status |= HAL_TIM_Base_Start_IT(&htim2);    // safety watchdog timer
  status |= HAL_TIM_Base_Start_IT(&htim4);    // position update trigger timer
  status |= HAL_TIM_Base_Start(&htim6);       // time keeper timer

  status |= HAL_ADCEx_InjectedStart(&hadc1);
  status |= HAL_ADCEx_InjectedStart(&hadc2);

  if (status != HAL_OK) {
    while (1) {
      // error loop
    }
  }

  Encoder_triggerUpdate(&controller->encoder);

  HAL_Delay(100);
  PowerStage_calibratePhaseCurrentOffset(&controller->powerstage);

  if (controller->mode == MODE_DISABLED) {
    controller->mode = MODE_IDLE;
    controller->error = ERROR_NO_ERROR;
  }
}

ErrorCode MotorController_getError(MotorController *controller) {
  return controller->error;
}

Mode MotorController_getMode(MotorController *controller) {
  return controller->mode;
}

void MotorController_setMode(MotorController *controller, Mode mode) {
  switch (mode) {
    case MODE_DISABLED:
      PowerStage_disable(&controller->powerstage);
      break;

    case MODE_IDLE:
      PowerStage_enable(&controller->powerstage);
      controller->error = ERROR_NO_ERROR;
      break;

    case MODE_CALIBRATION:
      PowerStage_enable(&controller->powerstage);
      break;

    case MODE_TORQUE:
    case MODE_VELOCITY:
    case MODE_POSITION:
    case MODE_DEBUG:
    case MODE_OPEN_VDQ:
    case MODE_OPEN_VALPHABETA:
    case MODE_OPEN_VABC:
    case MODE_OPEN_IDQ:
      if (controller->mode != MODE_IDLE) {
        PowerStage_disable(&controller->powerstage);
        controller->mode = MODE_DISABLED;
        controller->error = ERROR_INVALID_MODE_SWITCH;
        return;  // return directly, do not update mode
      }
      PowerStage_enable(&controller->powerstage);
      break;

    default:
      PowerStage_disable(&controller->powerstage);
      controller->mode = MODE_DISABLED;
      controller->error = ERROR_INVALID_MODE;
      return;  // return directly, do not update mode
  }
  controller->mode = mode;
}

void MotorController_setFluxAngle(MotorController *controller, float angle_setpoint, float voltage_setpoint) {
  float theta = wrapTo2Pi(angle_setpoint);
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);
  float v_q = 0.0;
  float v_d = voltage_setpoint;

  controller->current_controller.v_alpha_target = -sin_theta * v_q + cos_theta * v_d;
  controller->current_controller.v_beta_target =   cos_theta * v_q + sin_theta * v_d;
}

void MotorController_loadConfig(MotorController *controller) {
  EEPROMConfig *config = (EEPROMConfig *)FLASH_CONFIG_ADDRESS;

  controller->firmware_version                  = config->firmware_version;
  controller->device_id                         = config->device_id;

  controller->encoder.cpr                       = config->encoder_cpr;
  controller->encoder.position_offset           = config->encoder_position_offset;
  controller->encoder.velocity_filter_alpha     = config->encoder_velocity_filter_alpha;

  controller->powerstage.undervoltage_threshold = config->powerstage_undervoltage_threshold;
  controller->powerstage.overvoltage_threshold  = config->powerstage_overvoltage_threshold;

  controller->motor.pole_pairs                  = config->motor_pole_pairs;
  controller->motor.kv_rating                   = config->motor_kv_rating;
  controller->motor.flux_angle_offset           = config->motor_flux_angle_offset;

  controller->current_controller.current_filter_alpha   =   config->current_controller_current_filter_alpha;
  controller->current_controller.i_q_kp         = config->current_controller_i_q_kp;
  controller->current_controller.i_q_ki         = config->current_controller_i_q_ki;
  controller->current_controller.i_d_kp         = config->current_controller_i_d_kp;
  controller->current_controller.i_d_ki         = config->current_controller_i_d_ki;

  controller->position_controller.position_kp   = config->position_controller_position_kp;
  controller->position_controller.position_ki   = config->position_controller_position_ki;
  controller->position_controller.position_kd   = config->position_controller_position_kd;
  controller->position_controller.torque_limit_upper    = config->position_controller_torque_limit_upper;
  controller->position_controller.torque_limit_lower    = config->position_controller_torque_limit_lower;
  controller->position_controller.velocity_limit_upper  = config->position_controller_velocity_limit_upper;
  controller->position_controller.velocity_limit_lower  = config->position_controller_velocity_limit_lower;
  controller->position_controller.position_limit_upper  = config->position_controller_position_limit_upper;
  controller->position_controller.position_limit_lower  = config->position_controller_position_limit_lower;
}

uint32_t MotorController_storeConfig(MotorController *controller) {
  EEPROMConfig config;

  config.firmware_version                     = controller->firmware_version;
  config.device_id                            = controller->device_id;

  config.encoder_cpr                          = controller->encoder.cpr;
  config.encoder_position_offset              = controller->encoder.position_offset;
  config.encoder_velocity_filter_alpha        = controller->encoder.velocity_filter_alpha;

  config.powerstage_undervoltage_threshold    = controller->powerstage.undervoltage_threshold;
  config.powerstage_overvoltage_threshold     = controller->powerstage.overvoltage_threshold;

  config.motor_pole_pairs                     = controller->motor.pole_pairs;
  config.motor_kv_rating                      = controller->motor.kv_rating;
  config.motor_flux_angle_offset              = controller->motor.flux_angle_offset;

  config.current_controller_current_filter_alpha  = controller->current_controller.current_filter_alpha;
  config.current_controller_i_q_kp            = controller->current_controller.i_q_kp;
  config.current_controller_i_q_ki            = controller->current_controller.i_q_ki;
  config.current_controller_i_d_kp            = controller->current_controller.i_d_kp;
  config.current_controller_i_d_ki            = controller->current_controller.i_d_ki;

  config.position_controller_position_kp      = controller->position_controller.position_kp;
  config.position_controller_position_ki      = controller->position_controller.position_ki;
  config.position_controller_position_kd      = controller->position_controller.position_kd;
  config.position_controller_torque_limit_upper       = controller->position_controller.torque_limit_upper;
  config.position_controller_torque_limit_upper       = controller->position_controller.torque_limit_lower;
  config.position_controller_velocity_limit_upper     = controller->position_controller.velocity_limit_upper;
  config.position_controller_velocity_limit_lower     = controller->position_controller.velocity_limit_lower;
  config.position_controller_position_limit_upper     = controller->position_controller.position_limit_upper;
  config.position_controller_position_limit_lower     = controller->position_controller.position_limit_lower;

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
    return HAL_FLASH_GetError();
  }

  /* Program the user Flash area word by word*/
  for (uint16_t i=0; i<sizeof(config)/sizeof(uint64_t); i+=1) {
    uint64_t buf = (uint64_t)*(((uint64_t *)(&config)) + i);

    uint32_t target_address = FLASH_CONFIG_ADDRESS + i*8;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, target_address, buf) != HAL_OK) {
      return HAL_FLASH_GetError();
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
    to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return HAL_OK;
}

float MotorController_getTorque(MotorController *controller) {
  return controller->position_controller.torque_measured;
}

float MotorController_getVelocity(MotorController *controller) {
  return controller->position_controller.velocity_measured;
}

float MotorController_getPosition(MotorController *controller) {
  return controller->position_controller.position_measured;
}

void MotorController_updateCommutation(MotorController *controller, ADC_HandleTypeDef *hadc) {
  float position_measured = Encoder_getRelativePosition(&controller->encoder);

  float theta = wrapTo2Pi((position_measured * (float)controller->motor.pole_pairs) - controller->motor.flux_angle_offset);
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);

  PowerStage_getPhaseCurrent(&controller->powerstage,
    &controller->current_controller.i_a_measured,
    &controller->current_controller.i_b_measured,
    &controller->current_controller.i_c_measured);

  CurrentController_update(&controller->current_controller,
      controller->mode,
      sin_theta,
      cos_theta,
      controller->powerstage.bus_voltage_measured);

  if (controller->mode != MODE_IDLE) {
    PowerStage_setBridgeOutput(&controller->powerstage,
      controller->current_controller.v_a_setpoint,
      controller->current_controller.v_b_setpoint,
      controller->current_controller.v_c_setpoint);
  }
  else {
    PowerStage_setBridgeOutput(&controller->powerstage,
      0,
      0,
      0);
  }
}

void MotorController_triggerPositionUpdate(MotorController *controller) {
  if (controller->mode == MODE_DISABLED
      || controller->mode == MODE_IDLE) {
    PowerStage_disable(&controller->powerstage);
  }
  else if (controller->mode == MODE_CALIBRATION
      || controller->mode == MODE_TORQUE
      || controller->mode == MODE_VELOCITY
      || controller->mode == MODE_POSITION
      || controller->mode == MODE_OPEN_VDQ
      || controller->mode == MODE_OPEN_VALPHABETA
      || controller->mode == MODE_OPEN_VABC
      || controller->mode == MODE_OPEN_IDQ) {
    PowerStage_enable(&controller->powerstage);
  }
  else {
    MotorController_setMode(controller, MODE_DISABLED);
    controller->error = ERROR_INVALID_MODE;
  }

  Encoder_triggerUpdate(&controller->encoder);
}

void MotorController_updatePositionReading(MotorController *controller) {
  Encoder_update(&controller->encoder);

  PowerStage_getBusVoltage(&controller->powerstage);

  controller->position_controller.position_measured = Encoder_getPosition(&controller->encoder);
  controller->position_controller.velocity_measured = Encoder_getVelocity(&controller->encoder);
  controller->position_controller.torque_measured = (8.3 * controller->current_controller.i_q_measured) / (float)controller->motor.kv_rating;
}

void MotorController_updatePositionController(MotorController *controller) {
  PositionController_update(&controller->position_controller);

  if (controller->mode != MODE_OPEN_IDQ) {
    controller->current_controller.i_q_target = (controller->position_controller.torque_setpoint * (float)controller->motor.kv_rating) / 8.3;
    controller->current_controller.i_d_target = 0;
  }
}

void MotorController_updateService(MotorController *controller) {
  if (controller->mode == MODE_CALIBRATION) {
    MotorController_runCalibrationSequence(controller);
    return;
  }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
  }
  else {
    // red
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
  }
}

void MotorController_runCalibrationSequence(MotorController *controller) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);    // green LED
  MotorController_setMode(controller, MODE_CALIBRATION);

  // open loop calibration
  float prev_v_alpha_target = controller->current_controller.v_alpha_target;
  float prev_v_beta_target = controller->current_controller.v_beta_target;

  float flux_angle_setpoint = 0;
  float voltage_setpoint = 0.2;

  MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
  HAL_Delay(100);
  PowerStage_enable(&controller->powerstage);
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
      HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 10);
    }
  }


  float start_position = Encoder_getPosition(&controller->encoder);

  // move one electrical revolution forward
  for (int16_t i=0; i<=500; i+=1) {
    flux_angle_setpoint = (i / 500.0f) * (2*M_PI);

    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);
  }
  HAL_Delay(500);

  float end_position = Encoder_getPosition(&controller->encoder);

  for (int16_t i=500; i>=0; i-=1) {
    flux_angle_setpoint = (i / 500.0f) * (2*M_PI);
    MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);
  }

  flux_angle_setpoint = 0;
  MotorController_setFluxAngle(controller, flux_angle_setpoint, voltage_setpoint);
  HAL_Delay(500);

  start_position = 0.5 * Encoder_getPosition(&controller->encoder) + 0.5 * start_position;
  HAL_Delay(500);

  // release motor
  PowerStage_disable(&controller->powerstage);

  controller->current_controller.v_alpha_target = prev_v_alpha_target;
  controller->current_controller.v_beta_target = prev_v_beta_target;

  float delta_position = end_position - start_position;

  {
    char str[128];
    sprintf(str, "initial encoder angle: %f\r\n", start_position);
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 10);
    sprintf(str, "end encoder angle: %f\r\n", end_position);
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 10);
    sprintf(str, "delta angle: %f\r\n", delta_position);
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 10);
  }


  if (fabsf(delta_position) < 0.1) {
    // motor did not rotate
    HAL_UART_Transmit(&huart3, (uint8_t *)"ERROR: motor not rotating\r\n", strlen("ERROR: motor not rotating\r\n"), 10);
  }

  if (fabsf(fabsf(delta_position)*controller->motor.pole_pairs-(2*M_PI)) > 0.5f) {
    HAL_UART_Transmit(&huart3, (uint8_t *)"ERROR: motor pole pair mismatch\r\n", strlen("ERROR: motor pole pair mismatch\r\n"), 10);
  }


  // set electrical angle
  controller->motor.flux_angle_offset = wrapTo2Pi(start_position * controller->motor.pole_pairs);

  {
    char str[128];
    sprintf(str, "offset angle: %f\r\n", controller->motor.flux_angle_offset);
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 10);
  }

  MotorController_storeConfig(controller);

  HAL_Delay(1000);

  MotorController_setMode(controller, MODE_IDLE);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);    // green LED
}

void MotorController_handleCANMessage(MotorController *controller, CAN_Frame *rx_frame) {
  uint16_t device_id = (rx_frame->id) & 0b1111;
  if (device_id && device_id != controller->device_id) {
    return;
  }

  uint16_t func_id = (rx_frame->id) >> 4;
  uint8_t is_get_request = rx_frame->frame_type == CAN_FRAME_REMOTE || rx_frame->size == 0;

  if (is_get_request) {
    CAN_Frame tx_frame;

    tx_frame.id = rx_frame->id;
    tx_frame.id_type = CAN_ID_STANDARD;
    tx_frame.frame_type = CAN_FRAME_DATA;
    tx_frame.size = 8;

    switch (func_id) {
      case CAN_ID_ESTOP:
        MotorController_setMode(controller, MODE_DISABLED);
        tx_frame.size = 1;
        *((uint8_t *)tx_frame.data) = 0xAC;
        break;
      case CAN_ID_ID:
        tx_frame.size = 1;
        *((uint8_t *)tx_frame.data) = controller->device_id;
        break;
      case CAN_ID_VERSION:
        tx_frame.size = 1;
        *((uint8_t *)tx_frame.data) = controller->firmware_version;
        break;
      case CAN_ID_SAFETY:
        tx_frame.size = 1;
        *((uint8_t *)tx_frame.data) = controller->error;
        break;
      case CAN_ID_PING:
        tx_frame.size = 1;
        *((uint8_t *)tx_frame.data) = controller->device_id;
        break;
      case CAN_ID_MODE:
        tx_frame.size = 1;
        *((uint8_t *)tx_frame.data) = MotorController_getMode(controller);
        break;
      case CAN_ID_ENCODER_CPR:
        tx_frame.size = 4;
        *((uint32_t *)tx_frame.data) = controller->encoder.cpr;
        break;
      case CAN_ID_ENCODER_POSITION_OFFSET:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->encoder.position_offset;
        break;
      case CAN_ID_ENCODER_VELOCITY_FILTER_ALPHA:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->encoder.velocity_filter_alpha;
        break;
      case CAN_ID_ENCODER_N_ROTATIONS:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->encoder.n_rotations;
        break;
      case CAN_ID_ENCODER_POSITION_RELATIVE:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->encoder.position_relative;
        break;
      case CAN_ID_ENCODER_POSITION_RAW:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->encoder.position_raw;
        break;
      case CAN_ID_ENCODER_POSITION:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->encoder.position;
        break;
      case CAN_ID_ENCODER_VELOCITY:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->encoder.velocity;
        break;
      case CAN_ID_POWERSTAGE_VOLTAGE_THREASHOLD:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->powerstage.undervoltage_threshold;
        *((float *)tx_frame.data + 1) = controller->powerstage.overvoltage_threshold;
        break;
      case CAN_ID_POWERSTAGE_ADC_READING_RAW_A_B_C:
        tx_frame.size = 6;
        *((uint16_t *)tx_frame.data) = controller->powerstage.adc_reading_raw[0];
        *((uint16_t *)tx_frame.data + 1) = controller->powerstage.adc_reading_raw[1];
        *((uint16_t *)tx_frame.data + 2) = controller->powerstage.adc_reading_raw[2];
        break;
      case CAN_ID_POWERSTAGE_ADC_READING_OFFSET_A_B_C:
        tx_frame.size = 6;
        *((int16_t *)tx_frame.data) = controller->powerstage.adc_reading_offset[0];
        *((int16_t *)tx_frame.data + 1) = controller->powerstage.adc_reading_offset[1];
        *((int16_t *)tx_frame.data + 2) = controller->powerstage.adc_reading_offset[2];
        break;
      case CAN_ID_POWERSTAGE_BUS_VOLTAGE:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->powerstage.bus_voltage_measured;
        break;
      case CAN_ID_MOTOR_POLE_PAIRS:
        tx_frame.size = 4;
        *((uint32_t *)tx_frame.data) = controller->motor.pole_pairs;
        break;
      case CAN_ID_MOTOR_KV_RATING:
        tx_frame.size = 4;
        *((uint32_t *)tx_frame.data) = controller->motor.kv_rating;
        break;
      case CAN_ID_MOTOR_FLUX_ANGLE_OFFSET:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->motor.flux_angle_offset;
        break;
      case CAN_ID_CURRENT_CONTROLLER_CURRENT_FILTER_ALPHA:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->current_controller.current_filter_alpha;
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_Q_KP_KI:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.i_q_kp;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_q_ki;
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_D_KP_KI:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.i_d_kp;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_d_ki;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_A_TARGET_I_A_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_a_target;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_a_measured;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_B_TARGET_I_B_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_b_target;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_b_measured;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_C_TARGET_I_C_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_c_target;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_c_measured;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_A_V_B_SETPOINT:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_a_setpoint;
        *((float *)tx_frame.data + 1) = controller->current_controller.v_b_setpoint;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_C_SETPOINT:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->current_controller.v_c_setpoint;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_ALPHA_TARGET_I_ALPHA_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_alpha_target;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_alpha_measured;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_BETA_TARGET_I_BETA_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_beta_target;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_beta_measured;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_ALPHA_V_BETA_SETPOINT:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_alpha_setpoint;
        *((float *)tx_frame.data + 1) = controller->current_controller.v_beta_setpoint;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_Q_V_D_TARGET:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_q_target;
        *((float *)tx_frame.data + 1) = controller->current_controller.v_d_target;
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_Q_V_D_SETPOINT:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.v_q_setpoint;
        *((float *)tx_frame.data + 1) = controller->current_controller.v_d_setpoint;
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_Q_TARGET_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.i_q_target;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_q_measured;
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_D_TARGET_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.i_d_target;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_d_measured;
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_SETPOINT:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.i_q_setpoint;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_d_setpoint;
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_INTEGRATOR:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->current_controller.i_q_integrator;
        *((float *)tx_frame.data + 1) = controller->current_controller.i_d_integrator;
        break;
      case CAN_ID_POSITION_CONTROLLER_KP_KI:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->position_controller.position_kp;
        *((float *)tx_frame.data + 1) = controller->position_controller.position_ki;
        break;
      case CAN_ID_POSITION_CONTROLLER_KD:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->position_controller.position_kd;
        break;
      case CAN_ID_POSITION_CONTROLLER_TORQUE_LIMIT:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->position_controller.torque_limit_lower;
        *((float *)tx_frame.data + 1) = controller->position_controller.torque_limit_upper;
        break;
      case CAN_ID_POSITION_CONTROLLER_VELOCITY_LIMIT:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->position_controller.velocity_limit_lower;
        *((float *)tx_frame.data + 1) = controller->position_controller.velocity_limit_upper;
        break;
      case CAN_ID_POSITION_CONTROLLER_POSITION_LIMIT:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->position_controller.position_limit_lower;
        *((float *)tx_frame.data + 1) = controller->position_controller.position_limit_upper;
        break;
      case CAN_ID_POSITION_CONTROLLER_TORQUE_TARGET_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->position_controller.torque_target;
        *((float *)tx_frame.data + 1) = controller->position_controller.torque_measured;
        break;
      case CAN_ID_POSITION_CONTROLLER_TORQUE_SETPOINT:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->position_controller.torque_setpoint;
        break;
      case CAN_ID_POSITION_CONTROLLER_VELOCITY_TARGET_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->position_controller.velocity_target;
        *((float *)tx_frame.data + 1) = controller->position_controller.velocity_measured;
        break;
      case CAN_ID_POSITION_CONTROLLER_VELOCITY_SETPOINT:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->position_controller.velocity_setpoint;
        break;
      case CAN_ID_POSITION_CONTROLLER_POSITION_TARGET_MEASURED:
        tx_frame.size = 8;
        *((float *)tx_frame.data) = controller->position_controller.position_target;
        *((float *)tx_frame.data + 1) = controller->position_controller.position_measured;
        break;
      case CAN_ID_POSITION_CONTROLLER_POSITION_SETPOINT:
        tx_frame.size = 4;
        *((float *)tx_frame.data) = controller->position_controller.position_setpoint;
        break;
    }
    CAN_putTxFrame(&hfdcan1, &tx_frame);
  }
  else {
    switch (func_id) {
      case CAN_ID_ESTOP:
        MotorController_setMode(controller, MODE_DISABLED);
        break;
      case CAN_ID_ID:
        controller->device_id = *((uint8_t *)rx_frame->data);
        break;
      case CAN_ID_FLASH:
        if (*((uint8_t *)rx_frame->data)) {
          MotorController_storeConfig(controller);
        }
        else {
          MotorController_loadConfig(controller);
        }
        break;
      case CAN_ID_HEARTBEAT:
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        break;
      case CAN_ID_MODE:
        MotorController_setMode(controller, (Mode)*((uint8_t *)rx_frame->data));
        break;
      case CAN_ID_ENCODER_CPR:
        controller->encoder.cpr = *((uint32_t *)rx_frame->data);
        break;
      case CAN_ID_ENCODER_POSITION_OFFSET:
        controller->encoder.position_offset = *((float *)rx_frame->data);
        break;
      case CAN_ID_ENCODER_VELOCITY_FILTER_ALPHA:
        controller->encoder.velocity_filter_alpha = *((float *)rx_frame->data);
        break;
      case CAN_ID_POWERSTAGE_VOLTAGE_THREASHOLD:
        controller->powerstage.undervoltage_threshold = *((float *)rx_frame->data);
        controller->powerstage.overvoltage_threshold = *((float *)rx_frame->data + 1);
        break;
      case CAN_ID_CURRENT_CONTROLLER_CURRENT_FILTER_ALPHA:
        controller->current_controller.current_filter_alpha = *((float *)rx_frame->data);
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_Q_KP_KI:
        controller->current_controller.i_q_kp = *((float *)rx_frame->data);
        controller->current_controller.i_q_ki = *((float *)rx_frame->data + 1);
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_D_KP_KI:
        controller->current_controller.i_d_kp = *((float *)rx_frame->data);
        controller->current_controller.i_d_ki = *((float *)rx_frame->data + 1);
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_A_TARGET_I_A_MEASURED:
        controller->current_controller.v_a_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_B_TARGET_I_B_MEASURED:
        controller->current_controller.v_b_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_C_TARGET_I_C_MEASURED:
        controller->current_controller.v_c_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_ALPHA_TARGET_I_ALPHA_MEASURED:
        controller->current_controller.v_alpha_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_BETA_TARGET_I_BETA_MEASURED:
        controller->current_controller.v_beta_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_CURRENT_CONTROLLER_V_Q_V_D_TARGET:
        controller->current_controller.v_q_target = *((float *)rx_frame->data);
        controller->current_controller.v_d_target = *((float *)rx_frame->data + 1);
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_Q_TARGET_MEASURED:
        controller->current_controller.i_q_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_CURRENT_CONTROLLER_I_D_TARGET_MEASURED:
        controller->current_controller.i_d_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_POSITION_CONTROLLER_KP_KI:
        controller->position_controller.position_kp = *((float *)rx_frame->data);
        controller->position_controller.position_ki = *((float *)rx_frame->data + 1);
        break;
      case CAN_ID_POSITION_CONTROLLER_KD:
        controller->position_controller.position_kd = *((float *)rx_frame->data);
        break;
      case CAN_ID_POSITION_CONTROLLER_TORQUE_LIMIT:
        controller->position_controller.torque_limit_lower = *((float *)rx_frame->data);
        controller->position_controller.torque_limit_upper = *((float *)rx_frame->data + 1);
        break;
      case CAN_ID_POSITION_CONTROLLER_VELOCITY_LIMIT:
        controller->position_controller.velocity_limit_lower = *((float *)rx_frame->data);
        controller->position_controller.velocity_limit_upper = *((float *)rx_frame->data + 1);
        break;
      case CAN_ID_POSITION_CONTROLLER_POSITION_LIMIT:
        controller->position_controller.position_limit_lower = *((float *)rx_frame->data);
        controller->position_controller.position_limit_upper = *((float *)rx_frame->data + 1);
        break;
      case CAN_ID_POSITION_CONTROLLER_TORQUE_TARGET_MEASURED:
        controller->position_controller.torque_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_POSITION_CONTROLLER_VELOCITY_TARGET_MEASURED:
        controller->position_controller.velocity_target = *((float *)rx_frame->data);
        break;
      case CAN_ID_POSITION_CONTROLLER_POSITION_TARGET_MEASURED:
        controller->position_controller.position_target = *((float *)rx_frame->data);
        break;
    }
  }
}

