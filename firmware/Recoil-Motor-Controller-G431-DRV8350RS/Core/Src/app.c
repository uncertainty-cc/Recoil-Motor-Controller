/*
 * aoo.c
 *
 *  Created on: Oct 13, 2022
 *      Author: TK
 */

#include "app.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern CORDIC_HandleTypeDef hcordic;
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

MotorController controller;

float user_input_pot;
uint8_t user_input_button;
uint8_t user_output_led;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  CAN_Frame rx_frame;
  CAN_getRxFrame(&hfdcan1, &rx_frame);
  MotorController_handleCANMessage(&controller, &rx_frame);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);
    MotorController_updateCommutation(&controller, &hadc1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0);
  }
  else if (htim == &htim2) {
    #if SAFETY_WATCHDOG_ENABLED == 1
    if (controller.mode != MODE_IDLE && controller.mode != MODE_CALIBRATION) {
      MotorController_setMode(&controller, MODE_DISABLED);
      controller.error = ERROR_HEARTBEAT_TIMEOUT;
    }
    #endif
  }
  else if (htim == &htim4) {
    MotorController_triggerPositionUpdate(&controller);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 1);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
  MotorController_updatePositionReading(&controller);

  /* ====== Start user APP code ====== */
//  controller.position_controller.position_target = APP_getUserPot() * M_PI;

  /* ====== End user APP code ====== */

  MotorController_updatePositionController(&controller);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);
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
  #if INITIAL_PROG
  APP_initFlashOption();
  #endif

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);    // green
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);    // blue
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);    // red

  MotorController_init(&controller);

  MotorController_setMode(&controller, MODE_IDLE);
  controller.position_controller.position_target = 0;
  HAL_Delay(3000);

//  MotorController_setMode(&controller, MODE_CALIBRATION);
//  MotorController_updateService(&controller);
//  MotorController_setMode(&controller, MODE_POSITION);

//  MotorController_setMode(&controller, MODE_IQD_OVERRIDE);
//  controller.current_controller.i_q_setpoint = 0.05;
//  controller.current_controller.i_d_setpoint = 0;

//  MotorController_setMode(&controller, MODE_TORQUE);
//  controller.position_controller.torque_target = 0;

  MotorController_setMode(&controller, MODE_POSITION);
//  controller.position_controller.velocity_target = 0;
}


void APP_main() {
  MotorController_updateService(&controller);

  if (controller.mode != MODE_IDLE) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);    // blue LED
  }
  else {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);    // blue LED
  }

//  controller.position_controller.torque_target = 0.001;

  controller.position_controller.position_target = 10 * ((HAL_GetTick() / 4000) % 2);


  char str[128];

  //    sprintf(str, "pos:%f\tvbus:%f\r\n",
  //        controller.position_controller.position_measured,
  //        controller.powerstage.bus_voltage_measured);

  //    sprintf(str, "prel:%f\tpos:%f\tvel:%f\n",
  //        Encoder_getRelativePosition(&controller.encoder),
  //        Encoder_getPosition(&controller.encoder),
  //        Encoder_getVelocity(&controller.encoder));

  //  sprintf(str, "ia:%f\tib:%f\tic:%f\r\n",
  //      controller.current_controller.i_a_measured,
  //      controller.current_controller.i_b_measured,
  //      controller.current_controller.i_c_measured);

  //  sprintf(str, "pos:%f\tva:%f\tvb:%f\tvc:%f\r\n",
  //      controller.position_controller.position_measured,
  //      controller.current_controller.v_a_setpoint,
  //      controller.current_controller.v_b_setpoint,
  //      controller.current_controller.v_c_setpoint);



//  sprintf(str, "iq_set:%f\tid_set:%f\tiq_mea:%f\tid_mea:%f\r\n",
//      controller.current_controller.i_q_setpoint * 1000,
//      controller.current_controller.i_d_setpoint * 1000,
//      controller.current_controller.i_q_measured * 1000,
//      controller.current_controller.i_d_measured * 1000);


//  sprintf(str, "t_tar:%f\tt_mea:%f\n",
//      controller.position_controller.torque_target * 1000,
//      controller.position_controller.torque_measured * 1000);



  sprintf(str, "p_tar:%f\tp_mea:%f\tv_tar:%f\tv_mea:%f\tv_set:%f\tv_lim:%f\tt_set:%f\tt_mea:%f\r\n",
      controller.position_controller.position_target,
      controller.position_controller.position_measured,
      controller.position_controller.velocity_target,
      controller.position_controller.velocity_measured,
      controller.position_controller.velocity_setpoint,
      controller.position_controller.velocity_limit,
      controller.position_controller.torque_setpoint * 1000,
      controller.position_controller.torque_measured * 1000);

//    sprintf(str, "p_tar:%f\tp_mea:%f\tdx:%f\taccel:%f\tvel_mea:%f\tvel_set:%f\tpos_set:%f\tt_tar:%f\r\n",
//        controller.position_controller.position_target,
//        controller.position_controller.position_measured,
//        controller.position_controller.dx,
//        controller.position_controller.accel * .5,
//        controller.position_controller.velocity_measured,
//        controller.position_controller.velocity_setpoint,
//        controller.position_controller.position_setpoint,
//        controller.position_controller.torque_target*5 + 5);


  HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 1000);
  HAL_Delay(5);
}

