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
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart2;

MotorController controller;

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
    if (controller.mode != MODE_DISABLED && controller.mode != MODE_IDLE && controller.mode != MODE_CALIBRATION) {
      MotorController_setMode(&controller, MODE_DAMPING);
      SET_BITS(controller.error, ERROR_WATCHDOG_TIMEOUT);
    }
    #endif
  }
  else if (htim == &htim8) {
    if (controller.fast_frame_frequency != 0) {
      uint32_t func_id = FUNC_USR_FAST_FRAME_0;
      CAN_Frame tx_frame;
      tx_frame.id = (func_id << 6) | controller.device_id;
      tx_frame.id_type = CAN_ID_STANDARD;
      tx_frame.frame_type = CAN_FRAME_DATA;
      tx_frame.size = 8;
      *((float *)tx_frame.data + 0) = PositionController_getPositionMeasured(&controller.position_controller);
      *((float *)tx_frame.data + 1) = PositionController_getVelocityMeasured(&controller.position_controller);
      if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) >= 2) {
        CAN_putTxFrame(&hfdcan1, &tx_frame);
      }
    }
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  // do nothing here
}

/**
 * Procedure following G431 User Manual Section 4.4.2 Option bytes programming
 *
 */
void APP_initFlashOption() {
  // 1. Unlock the FLASH_CR with the LOCK clearing sequence
  // Check that no Flash memory operation is on going by checking the BSY bit in the Flash status register (FLASH_SR).
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}
  HAL_FLASH_Unlock();

  // 2. Unlock the FLASH Option Byte with the LOCK clearing sequence
  // Check that no Flash memory operation is on going by checking the BSY bit in the Flash status register (FLASH_SR).
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}
  HAL_FLASH_OB_Unlock();

  // 3. program OPTR
  FLASH->OPTR = 0xFBEFF8AAU;  // default to boot from flash

  // 4. Set the Options Start bit OPTSTRT in the Flash control register (FLASH_CR).
  SET_BITS(FLASH->CR, FLASH_CR_OPTSTRT);

  // 4.1 clear status register
  SET_BITS(FLASH->SR, FLASH_SR_OPTVERR | FLASH_SR_RDERR);

  // 5. Wait for the BSY bit to be cleared.
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  // 6. Lock Flash
  // If LOCK is set by software, OPTLOCK is automatically set too
  HAL_FLASH_Lock();

  // 7. reload the new settings
  // seems this line will cause error when put before FLASH_Lock(), which will then corrupt all Flash settings
  // so putting it here
  // can also comment out this line and just power cycle to update the flash settings
  HAL_FLASH_OB_Launch();
}

void APP_init() {
  #if FIRST_TIME_BOOTUP
    APP_initFlashOption();
    MotorController_storeConfig(&controller);
    while (1) {}
  #endif

  MotorController_init(&controller);


  HAL_Delay(1000);
//
//  MotorController_setMode(&controller, MODE_DAMPING);
//  controller.current_controller.i_q_target = 0;
//  controller.current_controller.i_d_target = 0;
//  MotorController_setMode(&controller, MODE_CURRENT);
//
////
//  MotorController_setMode(&controller, MODE_VALPHABETA_OVERRIDE);
//  controller.position_controller.position_target = 0;
//  MotorController_setMode(&controller, MODE_POSITION);

}

uint8_t APP_getUserButton() {
  return HAL_GPIO_ReadPin(GPIO_BUTTON_GPIO_Port, GPIO_BUTTON_Pin) ? 0 : 1;
}

float APP_getUserPot() {
  return HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) * ADC_READING_COEFFICIENT / 3.3;
}

float theta = 0;

void APP_main() {
  MotorController_updateService(&controller);

  char str[128];
  if (APP_getUserButton()) {
    MotorController_setMode(&controller, MODE_CALIBRATION);
  }

//    sprintf(str, "mode:%d\r\n", controller.mode);

//   initial status logging
//  sprintf(str, "p:%f\tv:%f\tvoltage:%f\tpot:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.position_controller.velocity_measured,
//      controller.powerstage.bus_voltage_measured,
//      APP_getUserPot());

//   current loop logging
//  sprintf(str, "iq_tar:%f\tiq_mea:%f\tiq_set:%f\ttor:%f\tvel:%f\r\n",
//      controller.current_controller.i_q_target * 100,
//      controller.current_controller.i_q_measured * 100,
//      controller.current_controller.i_q_setpoint * 100,
//      controller.position_controller.torque_setpoint * 50,
//      controller.position_controller.velocity_measured * 100);


//  // position loop logging
  sprintf(str, "p_mea:%f\tp_tar:%f\tiq_set:%f\tvel_mea:%f\r\n",
      controller.position_controller.position_measured,
      controller.position_controller.position_target,
      controller.current_controller.i_q_setpoint * 10,
      controller.position_controller.velocity_measured * 10);

  // torque testing
//  sprintf(str, "t_mea:%f\tiq_mea:%f\tiq_tar:%f\tt_tar:%f\tt_set:%f\r\n",
//        controller.position_controller.torque_measured,
//        controller.current_controller.i_q_measured * 100,
//        controller.current_controller.i_q_target * 100,
//        controller.position_controller.torque_target,
//        controller.position_controller.torque_setpoint);


//  sprintf(str, "valpha:%f\tvbeta:%f\tvq:%f\tvd:%f\r\n",
//        controller.current_controller.v_alpha_setpoint * 10,
//        controller.current_controller.v_beta_setpoint * 10,
//        controller.current_controller.v_q_setpoint * 10,
//        controller.current_controller.v_d_setpoint * 10);

//  sprintf(str, "pos:%f\tia:%f\tib:%f\tic:%f\r\n",
//      Encoder_getPosition(&controller.encoder),
//      controller.current_controller.i_a_measured,
//      controller.current_controller.i_b_measured,
//      controller.current_controller.i_c_measured);

//    sprintf(str, "sizeof motorcontroller: %u\r\n", sizeof(MotorController));


  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
  HAL_Delay(10);
}

