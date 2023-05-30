/*
 * app.c
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */


#include "app.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c1;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;

MotorController controller;

uint32_t counter;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  CAN_Frame rx_frame;
  CAN_getRxFrame(&hfdcan1, &rx_frame);
  MotorController_handleCANMessage(&controller, &rx_frame);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) {
    MotorController_update(&controller);

    /* ====== Start user APP code ====== */
  //  controller.position_controller.position_target = APP_getUserPot() * M_PI;

    /* ====== End user APP code ====== */
  }
  else if (htim == &htim2) {
    #if SAFETY_WATCHDOG_ENABLED
    // watchdog time: 1000ms
    if (controller.mode != MODE_IDLE && controller.mode != MODE_CALIBRATION) {
      MotorController_setMode(&controller, MODE_DAMPING);
      SET_BITS(controller.error, ERROR_WATCHDOG_TIMEOUT);
    }
    #endif
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  // do nothing here
}

uint8_t APP_getUserButton() {
  return HAL_GPIO_ReadPin(GPIO_BUTTON_GPIO_Port, GPIO_BUTTON_Pin) ? 0 : 1;
}

float APP_getUserPot() {
  return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) * ADC_READING_COEFFICIENT / 3.3;
}

void APP_initFlashOption() {
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();

  FLASH->OPTR = 0xFBEFF8AA;  // default to boot from flash

  //SET_BITS(FLASH->CR, FLASH_CR_OPTSTRT);
  FLASH->CR |= FLASH_CR_OPTSTRT;

  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  HAL_FLASH_Lock();
  HAL_FLASH_OB_Launch();  // reload the new settings
}

void APP_init() {
  #if FIRST_TIME_BOOTUP
    APP_initFlashOption();
    MotorController_storeConfig(&controller);
    while (1) {}
  #endif

  MotorController_init(&controller);


  HAL_Delay(1000);

//  MotorController_setMode(&controller, MODE_DAMPING);

//  MotorController_setMode(&controller, MODE_CALIBRATION);
//  MotorController_updateService(&controller);

//  controller.current_controller.v_q_target = 0.6;
////  controller.current_controller.v_q_target = 1.2;
//  controller.current_controller.v_d_target = 0;
//  MotorController_setMode(&controller, MODE_VQD_OVERRIDE);

//  controller.current_controller.i_q_target = 0.1;
//  controller.current_controller.i_q_target = 4.f;
//
//  controller.current_controller.i_q_target = 2.f;    // M6C12
//  controller.current_controller.i_q_target = 0.6f;      // 5010 110KV
//  controller.current_controller.i_d_target = 0.;
//  MotorController_setMode(&controller, MODE_CURRENT);

//  controller.position_controller.position_target = 0.0f;
//  MotorController_setMode(&controller, MODE_POSITION);
}

void APP_main() {
  MotorController_updateService(&controller);

//  counter += 1;


//  controller.current_controller.v_q_target = 0.6;
////  controller.current_controller.v_q_target = 1.2;
//  controller.current_controller.v_d_target = 0;
//  MotorController_setMode(&controller, MODE_VQD_OVERRIDE);
//  MotorController_setMode(&controller, MODE_CURRENT);
//  if (counter > 1000) {
////    controller.current_controller.i_q_target = 1.f;
//    controller.position_controller.position_target = 0.0f;
//  }
//  if (counter > 2000) {
//    controller.position_controller.position_target = 3.0f * 14;
//    counter = 0;
//  }


  char str[128];

  if (APP_getUserButton()) {
    MotorController_setMode(&controller, MODE_CALIBRATION);
  }

//    sprintf(str, "p_c:%f\tp_raw:%f\tp_com:%f\r\n",
//        controller.encoder.position,
//        (((float)controller.encoder.position_raw / (float)controller.encoder.cpr) + controller.encoder.n_rotations) * (M_2PI_F),
//        (((float)controller.encoder.position_raw / (float)controller.encoder.cpr) + controller.encoder.n_rotations) * (M_2PI_F)+ controller.encoder.flux_offset_table[(controller.encoder.position_raw > 0 ? controller.encoder.position_raw : controller.encoder.position_raw + controller.encoder.cpr/2) >> 5]);

  // initial status logging
//  sprintf(str, "p:%f\tv:%f\tvoltage:%f\tpot:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.position_controller.velocity_measured,
//      controller.powerstage.bus_voltage_measured,
//      APP_getUserPot());

  // current loop logging
//  sprintf(str, "iq_mea:%f\tid_mea:%f\tiq_tar:%f\tiq_set:%f\tvq_tar:%f\r\n",
//      controller.current_controller.i_q_measured * 100,
//      controller.current_controller.i_d_measured * 100,
//      controller.current_controller.i_q_target * 100,
//      controller.current_controller.i_q_setpoint * 100,
//      controller.current_controller.v_q_target);


  // position loop logging
  sprintf(str, "mea:%f\ttar:%f\tset:%f\tiq:%f\r\n",
      controller.position_controller.position_measured,
      controller.position_controller.position_target,
      controller.position_controller.position_setpoint,
      controller.current_controller.i_q_target * 10);


//  sprintf(str, "valpha:%f\tvbeta:%f\tvq:%f\tvd:%f\r\n",
//        controller.current_controller.v_alpha_setpoint * 10,
//        controller.current_controller.v_beta_setpoint * 10,
//        controller.current_controller.v_q_setpoint * 10,
//        controller.current_controller.v_d_setpoint * 10);

//  sprintf(str, "pos:%f\tva:%f\tvb:%f\tvc:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.current_controller.v_a_setpoint,
//      controller.current_controller.v_b_setpoint,
//      controller.current_controller.v_c_setpoint);


  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);

  HAL_Delay(1);

}

