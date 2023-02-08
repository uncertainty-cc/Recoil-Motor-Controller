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
    #if SAFETY_WATCHDOG_ENABLED == 1
    if (controller.mode != MODE_IDLE && controller.mode != MODE_CALIBRATION) {
      MotorController_setMode(&controller, MODE_ERROR);
      controller.error = ERROR_HEARTBEAT_TIMEOUT;
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

  while (1) {}
  #endif

  MotorController_init(&controller);

  /* after initialization, the motor controller will be in IDLE mode. */

//  HAL_Delay(3000);
//  MotorController_setMode(&controller, MODE_CALIBRATION);
//  MotorController_updateService(&controller);
  uint16_t tx_buffer[2];
  uint16_t rx_buffer[2];


  PowerStage_enableGateDriver(&controller.powerstage);

  tx_buffer[0] = (1 << 15) | (0x00 << 11);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

  tx_buffer[0] = (1 << 15) | (0x03 << 11);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);

  tx_buffer[0] = (1 << 15) | (0x04 << 11);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx_buffer, (uint8_t *)rx_buffer, 1, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);



  HAL_Delay(1000);

  MotorController_setMode(&controller, MODE_VABC_OVERRIDE);
  controller.current_controller.v_a_setpoint = 0;
  controller.current_controller.v_b_setpoint = 0;
  controller.current_controller.v_c_setpoint = 0;

//  MotorController_setMode(&controller, MODE_VQD_OVERRIDE);
//    controller.current_controller.v_q_target = 1;
//    controller.current_controller.v_d_target = 0;

//    MotorController_setMode(&controller, MODE_IQD_OVERRIDE);
//    controller.current_controller.i_q_target = 0.0001;
//    controller.current_controller.i_d_target = 0;

    //  MotorController_setMode(&controller, MODE_CURRENT);
//  controller.current_controller.i_q_target = 0.2;
//  controller.current_controller.i_d_target = 0;

//  controller.position_controller.torque_target = 0.01;
//  controller.position_controller.torque_target = 0.002;
//  MotorController_setMode(&controller, MODE_TORQUE);



//  controller.position_controller.position_target = 0;
//  MotorController_setMode(&controller, MODE_POSITION);


//  MotorController_updateService(&controller);
//  MotorController_setMode(&controller, MODE_POSITION);


//  MotorController_setMode(&controller, MODE_TORQUE);
//  controller.position_controller.torque_target = 0;
//  HAL_Delay(1000);
//  MotorController_setMode(&controller, MODE_POSITION);
//  controller.position_controller.position_target = 0;
}


void APP_main() {
  MotorController_updateService(&controller);

//  controller.position_controller.torque_target = 0.001;

//  controller.position_controller.position_target = 10 * ((HAL_GetTick() / 4000) % 2);


  char str[128];


//  sprintf(str, "pos:%f\tvbus:%f\r\n",
//      controller.position_controller.position_measured,
//      controller.powerstage.bus_voltage_measured);

//  sprintf(str, "pos:%f\tposm:%f\tpose:%f\n",
//      Encoder_getPositionMeasured(&controller.encoder),
//      Encoder_getPosition(&controller.encoder),
//      controller.debug_buffer);

  /* PowerStage current offset */
//  sprintf(str, "ia:%d\tib:%d\tic:%d\r\n",
//      controller.powerstage.adc_reading_offset[0],
//      controller.powerstage.adc_reading_offset[1],
//      controller.powerstage.adc_reading_offset[2]);

//    sprintf(str, "ia:%f\tib:%f\tic:%f\r\n",
//        controller.current_controller.i_a_measured * 1000,
//        controller.current_controller.i_b_measured * 1000,
//        controller.current_controller.i_c_measured * 1000);

  sprintf(str, "iq_tar:%f\tiq_set:%f\tiq_mea:%f\tid_set:%f\tid_mea:%f\r\n",
      controller.current_controller.i_q_target * 1000,
      controller.current_controller.i_q_setpoint * 1000,
      controller.current_controller.i_q_measured * 1000,
      controller.current_controller.i_d_setpoint * 1000,
      controller.current_controller.i_d_measured * 1000);

//  sprintf(str, "vq_tar:%f\tvq_set:%f\tvd_tar:%fvd_set:%f\t\r\n",
//      controller.current_controller.v_q_target,
//      controller.current_controller.v_q_setpoint,
//      controller.current_controller.v_d_target,
//      controller.current_controller.v_d_setpoint);

//    sprintf(str, "pos:%f\tva:%f\tvb:%f\tvc:%f\r\n",
//        controller.position_controller.position_measured,
//        controller.current_controller.v_a_setpoint,
//        controller.current_controller.v_b_setpoint,
//        controller.current_controller.v_c_setpoint);




//  sprintf(str, "t_tar:%f\tt_mea:%f\n",
//      controller.position_controller.torque_target * 1000,
//      controller.position_controller.torque_measured * 1000);


//
//  sprintf(str, "p_tar:%f\tp_mea:%f\tv_mea:%f\tt_tar:%f\tt_set:%f\tt_mea:%f\tiq:%f\r\n",
//      controller.position_controller.position_target,
//      controller.position_controller.position_measured,
//      controller.position_controller.velocity_measured,
//      controller.position_controller.torque_target * 1000,
//      controller.position_controller.torque_setpoint * 1000,
//      controller.position_controller.torque_measured * 1000,
//      controller.current_controller.i_q_setpoint);


  HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 1000);
//  HAL_Delay(5);
}

