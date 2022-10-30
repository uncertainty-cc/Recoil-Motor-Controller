/*
 * motor_controller_enum.h
 *
 *  Created on: Sep 8, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_CONTROLLER_CONF_H_
#define INC_MOTOR_CONTROLLER_CONF_H_

#define FIRMWARE_VERSION                0x00100002    // (MAJOR [7:5]) . (MINOR [4:2]) . (PATCH [1:0])

#define DEVICE_CAN_ID                   2

#define OVERWRITE_CONFIG                0

#define MAX_CALIBRATION_CURRENT         2.5



#define ADC_RESOLUTION                  4096    // 12-bit ADC
#define ADC_READING_COEFFICIENT         (3.3 / (float)ADC_RESOLUTION)
#define ADC_BUS_VOLTAGE_COEFFICIENT     (ADC_READING_COEFFICIENT * ((15. + 220.) / 15.))
// = ((3V3 / ADC_RESOLUTION) / opamp_factor) / R
#define ADC_OPAMP_CURRENT_COEFFICIENT   ((ADC_READING_COEFFICIENT / 16.) / 0.003) // convert ADC bits to Amps


typedef enum {
  CAN_ID_ESTOP              = 0x00U,
  CAN_ID_ID                 = 0x01U,
  CAN_ID_VERSION            = 0x02U,
  CAN_ID_HEARTBEAT          = 0x04U,

  // controller
  CAN_ID_MODE               = 0x10U,
  CAN_ID_FLASH              = 0x11U,

  CAN_ID_TORQUE_MEASURED    = 0x20U,
  CAN_ID_TORQUE_TARGET      = 0x21U,
  CAN_ID_VELOCITY_MEASURED  = 0x22U,
  CAN_ID_VELOCITY_TARGET    = 0x23U,
  CAN_ID_POSITION_MEASURED  = 0x24U,
  CAN_ID_POSITION_TARGET    = 0x25U,
  CAN_ID_POSITION_KP_KI     = 0x26U,
  CAN_ID_POSITION_KD        = 0x27U,
  CAN_ID_IQ_KP_KI           = 0x28U,
  CAN_ID_ID_KP_KI           = 0x29U,

  CAN_ID_BUS_VOLTAGE        = 0x30U,

  CAN_ID_MOTOR_SPEC         = 0x40U,
  CAN_ID_MOTOR_FLUX_OFFSET  = 0x41U,
  CAN_ID_ENCODER_N_ROTATION = 0x42U,

  CAN_ID_CURRENT_DQ         = 0x44U,
  CAN_ID_CURRENT_AB         = 0x45U,

  CAN_ID_CURRENTCONTROLLER_IQ = 0x50U,    // [i_q_target, i_q_measured]
  CAN_ID_CURRENTCONTROLLER_ID = 0x51U,    // [i_d_target, i_d_measured]
  CAN_ID_CURRENTCONTROLLER_VQ = 0x52U,    // [v_q_target]
  CAN_ID_CURRENTCONTROLLER_VD = 0x53U,    // [v_d_target]

  CAN_ID_PING               = 0x7FU
} CAN_ID_Types;


typedef enum {
  MODE_DISABLED           = 0x00U,
  MODE_IDLE               = 0x01U,

  MODE_CALIBRATION        = 0x05U,

  MODE_TORQUE             = 0x10U,
  MODE_VELOCITY           = 0x11U,
  MODE_POSITION           = 0x12U,

  MODE_OPEN_VDQ           = 0x22U,
  MODE_OPEN_VALPHABETA    = 0x23U,
  MODE_OPEN_VABC          = 0x24U,
  MODE_OPEN_IDQ           = 0x25U,

  MODE_DEBUG              = 0x80U,
} Mode;

typedef enum {
  ERROR_NO_ERROR          = 0x00U,
  ERROR_GENERAL           = 0x01U,
  ERROR_INVALID_MODE,
  ERROR_INVALID_MODE_SWITCH,
  ERROR_HEARTBEAT_TIMEOUT,
  ERROR_OVER_VOLTAGE,
  ERROR_OVER_CURRENT,
  ERROR_OVER_TEMPERATURE,
  ERROR_CAN_TX_FAULT,
  ERROR_I2C_FAULT,
} ErrorCode;

#endif /* INC_MOTOR_CONTROLLER_CONF_H_ */
