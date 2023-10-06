/*
 * aoo.c
 *
 *  Created on: Oct 13, 2022
 *      Author: TK
 */

#include "app.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
//extern OPAMP_HandleTypeDef hopamp1;
//extern OPAMP_HandleTypeDef hopamp2;
//extern OPAMP_HandleTypeDef hopamp3;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
extern UART_HandleTypeDef huart3;

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


//    MotorController_setMode(&controller, MODE_CALIBRATION);
//    MotorController_updateService(&controller);
//
//  MotorController_setMode(&controller, MODE_VQD_OVERRIDE);
//  controller.current_controller.v_q_target = 2;
//  controller.current_controller.v_d_target = 0;


//  MotorController_setMode(&controller, MODE_DAMPING);
//  HAL_Delay(1000);
//

  MotorController_setMode(&controller, MODE_CURRENT);
  controller.current_controller.i_q_target = 1.5f;
  controller.current_controller.i_d_target = 0.;

//  controller.position_controller.torque_target = 0.001;
//  MotorController_setMode(&controller, MODE_TORQUE);
//
//  controller.position_controller.position_target = 0;
//  MotorController_setMode(&controller, MODE_POSITION);
}


void APP_main() {
  MotorController_updateService(&controller);


  char str[128];

//    sprintf(str, "mode:%d\r\n", controller.mode);

//   initial status logging
//  sprintf(str, "p:%f\tv:%f\tvoltage:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.position_controller.velocity_measured,
//      controller.powerstage.bus_voltage_measured);

//   current loop logging
//  sprintf(str, "iq_mea:%f\tid_mea:%f\tiq_set:%f\tiq_tar:%f\vq_set:%f\tvel:%f\r\n",
//      controller.current_controller.i_q_measured,
//      controller.current_controller.i_d_measured,
//      controller.current_controller.i_q_setpoint,
//      controller.current_controller.i_q_target,
//      controller.current_controller.v_q_setpoint,
//      controller.position_controller.velocity_measured);


//  // position loop logging
//  sprintf(str, "p_mea:%f\tp_tar:%f\tiq_set:%f\tvel_mea:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.position_controller.position_target,
//    controller.current_controller.i_q_setpoint * 10,
//    controller.position_controller.velocity_measured * 10);

  // torque testing
//  sprintf(str, "pos:%f\tiq_mea:%f\tiq_tar:%f\ttorque:%f\r\n",
//        controller.position_controller.torque_measured,
//        controller.current_controller.i_q_measured * 100,
//        controller.current_controller.i_q_target * 100,
//        controller.position_controller.torque_setpoint);


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

    sprintf(str, "pos:%f\tia:%f\tib:%f\tic:%f\tvbus:%f\r\n",
        controller.position_controller.position_measured,
        controller.current_controller.i_a_measured,
        controller.current_controller.i_b_measured,
        controller.current_controller.i_c_measured,
        controller.powerstage.bus_voltage_measured);


  HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 1000);
//  HAL_Delay(10);
}

