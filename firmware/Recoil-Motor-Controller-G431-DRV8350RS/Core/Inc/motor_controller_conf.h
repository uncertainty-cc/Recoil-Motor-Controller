/*
 * motor_controller_enum.h
 *
 *  Created on: Sep 8, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_CONTROLLER_CONF_H_
#define INC_MOTOR_CONTROLLER_CONF_H_

#define FIRMWARE_VERSION                0x00200100    // (MAJOR [7:5]) . (MINOR [4:2]) . (PATCH [1:0])

#define DEVICE_CAN_ID                   10

#define FIRST_TIME_BOOTUP               0
#define LOAD_CONFIG_FROM_FLASH          0
#define LOAD_CALIBRATION_FROM_FLASH     0

#define SAFETY_WATCHDOG_ENABLED         0

#define CALIBRATION_CURRENT             3


#define ADC_RESOLUTION                  4096    // 12-bit ADC
#define ADC_READING_COEFFICIENT         (3.3f / (float)ADC_RESOLUTION)
#define ADC_BUS_VOLTAGE_COEFFICIENT     (ADC_READING_COEFFICIENT * ((10.f + 220.f) / 10.f))
// = ((3V3 / ADC_RESOLUTION) / opamp_factor) / R
#define ADC_OPAMP_CURRENT_COEFFICIENT   ((ADC_READING_COEFFICIENT / 16.f) / 0.003f) // convert ADC bits to Amps

// overwrite setting
#if FIRST_TIME_BOOTUP
#undef  OVERWRITE_PARAMETERS
#undef  OVERWRITE_CALIBRATION
#define OVERWRITE_PARAMETERS            1
#define OVERWRITE_CALIBRATION           1
#endif

typedef enum {
  CAN_ID_ESTOP              = 0x00U,
  CAN_ID_ID                 = 0x01U,
  CAN_ID_VERSION            = 0x02U,
  CAN_ID_SAFETY             = 0x03U,
  CAN_ID_FLASH              = 0x04U,

  // controller
  CAN_ID_MODE                                           = 0x06U,
  CAN_ID_STATUS                                         = 0x07U,


  CAN_ID_ENCODER_CPR                                    = 0x10U,
  CAN_ID_ENCODER_FILTER_BANDWIDTH                       = 0x11U,
  CAN_ID_ENCODER_POSITION_OFFSET                        = 0x12U,
  CAN_ID_ENCODER_N_ROTATIONS                            = 0x13U,
  CAN_ID_ENCODER_POSITION_RELATIVE                      = 0x14U,
  CAN_ID_ENCODER_POSITION                               = 0x15U,
  CAN_ID_ENCODER_VELOCITY                               = 0x16U,

  CAN_ID_POWERSTAGE_VOLTAGE_THREASHOLD                  = 0x20U,
  CAN_ID_POWERSTAGE_ADC_READING_RAW_A_B_C               = 0x21U,
  CAN_ID_POWERSTAGE_ADC_READING_OFFSET_A_B_C            = 0x22U,
  CAN_ID_POWERSTAGE_BUS_VOLTAGE                         = 0x23U,

  CAN_ID_MOTOR_POLE_PAIRS                               = 0x30U,
  CAN_ID_MOTOR_KV_RATING                                = 0x31U,
  CAN_ID_MOTOR_FLUX_ANGLE_OFFSET                        = 0x32U,

  CAN_ID_CURRENT_CONTROLLER_CURRENT_FILTER_ALPHA        = 0x40U,
  CAN_ID_CURRENT_CONTROLLER_I_Q_KP_KI                   = 0x41U,
  CAN_ID_CURRENT_CONTROLLER_I_D_KP_KI                   = 0x42U,
  CAN_ID_CURRENT_CONTROLLER_I_A_I_B_MEASURED            = 0x43U,
  CAN_ID_CURRENT_CONTROLLER_I_C_MEASURED                = 0x44U,
  CAN_ID_CURRENT_CONTROLLER_V_A_V_B_SETPOINT            = 0x45U,
  CAN_ID_CURRENT_CONTROLLER_V_C_SETPOINT                = 0x46U,
  CAN_ID_CURRENT_CONTROLLER_I_ALPHA_I_BETA_MEASURED     = 0x47U,
  CAN_ID_CURRENT_CONTROLLER_V_ALPHA_V_BETA_SETPOINT     = 0x48U,
  CAN_ID_CURRENT_CONTROLLER_V_Q_V_D_TARGET              = 0x49U,
  CAN_ID_CURRENT_CONTROLLER_V_Q_V_D_SETPOINT            = 0x4AU,
  CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_MEASURED            = 0x4BU,
  CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_TARGET              = 0x4CU,
  CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_SETPOINT            = 0x4DU,
  CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_INTEGRATOR          = 0x4EU,

  CAN_ID_POSITION_CONTROLLER_POSITION_KP_KI             = 0x50U,
  CAN_ID_POSITION_CONTROLLER_VELOCITY_KP_KI             = 0x51U,
  CAN_ID_POSITION_CONTROLLER_TORQUE_VELOCITY_LIMIT      = 0x52U,
  CAN_ID_POSITION_CONTROLLER_POSITION_LIMIT             = 0x54U,
  CAN_ID_POSITION_CONTROLLER_TORQUE_TARGET_MEASURED     = 0x55U,
  CAN_ID_POSITION_CONTROLLER_TORQUE_SETPOINT            = 0x56U,
  CAN_ID_POSITION_CONTROLLER_VELOCITY_TARGET_MEASURED   = 0x57U,
  CAN_ID_POSITION_CONTROLLER_VELOCITY_SETPOINT          = 0x58U,
  CAN_ID_POSITION_CONTROLLER_POSITION_TARGET_MEASURED   = 0x59U,
  CAN_ID_POSITION_CONTROLLER_POSITION_SETPOINT          = 0x5AU,

  CAN_ID_HEARTBEAT          = 0x7EU,
  CAN_ID_PING               = 0x7FU
} CAN_ID_Types;


typedef enum {
  // these are three safe modes
  MODE_DISABLED             = 0x00U,
  MODE_IDLE                 = 0x02U,

  MODE_CALIBRATION          = 0x05U,

  // these are closed-loop modes
  MODE_CURRENT              = 0x10U,
  MODE_TORQUE               = 0x11U,
  MODE_VELOCITY             = 0x12U,
  MODE_POSITION             = 0x13U,

  // these are open-loop modes
  MODE_VABC_OVERRIDE        = 0x20U,
  MODE_VALPHABETA_OVERRIDE  = 0x21U,
  MODE_VQD_OVERRIDE         = 0x22U,
  MODE_IQD_OVERRIDE         = 0x23U,

  MODE_DEBUG                = 0x80U,
} Mode;

typedef enum {
  ERROR_NO_ERROR                  = 0b0000000000000000U,
  ERROR_GENERAL                   = 0b0000000000000001U,
  ERROR_ESTOP                     = 0b0000000000000010U,
  ERROR_INITIALIZATION_ERROR      = 0b0000000000000100U,
  ERROR_CALIBRATION_ERROR         = 0b0000000000001000U,
  ERROR_POWERSTAGE_ERROR          = 0b0000000000010000U,
  ERROR_INVALID_MODE              = 0b0000000000100000U,
  ERROR_HEARTBEAT_TIMEOUT         = 0b0000000001000000U,
  ERROR_OVER_VOLTAGE              = 0b0000000010000000U,
  ERROR_OVER_CURRENT              = 0b0000000100000000U,
  ERROR_OVER_TEMPERATURE          = 0b0000001000000000U,
  ERROR_CAN_TX_FAULT              = 0b0000010000000000U,
  ERROR_I2C_FAULT                 = 0b0000100000000000U,
} ErrorCode;

#endif /* INC_MOTOR_CONTROLLER_CONF_H_ */
