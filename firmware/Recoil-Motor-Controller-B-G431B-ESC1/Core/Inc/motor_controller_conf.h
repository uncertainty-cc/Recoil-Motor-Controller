/*
 * motor_controller_enum.h
 *
 *  Created on: Sep 8, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_CONTROLLER_CONF_H_
#define INC_MOTOR_CONTROLLER_CONF_H_

#define FIRMWARE_VERSION                0x00300100    // (MAJOR [7:5]) . (MINOR [4:2]) . (PATCH [1:0])

#define DEVICE_CAN_ID                   2


#define FIRST_TIME_BOOTUP               0             // first time bootup: change Flash option byte, store config to Flash
#define LOAD_CONFIG_FROM_FLASH          0             // load config settings from Flash (everything except motor flux offset)
#define LOAD_CALIBRATION_FROM_FLASH     0             // load motor flux offset settings from Flash

#define SAFETY_WATCHDOG_ENABLED         0             // timeout

#define CALIBRATION_CURRENT             3             // calibration phase current
#define I2C_THROTTLE_COUNTER            2             // I2C packet rate = 20kHz / throttle_counter


#define ADC_RESOLUTION                  4096          // 12-bit ADC
#define ADC_READING_COEFFICIENT         (3.3f / (float)ADC_RESOLUTION)
#define ADC_BUS_VOLTAGE_COEFFICIENT     (ADC_READING_COEFFICIENT * ((10.f + 220.f) / 10.f))   // convert ADC bits to Volts
// = ((3V3 / ADC_RESOLUTION) / opamp_factor) / R
#define ADC_OPAMP_CURRENT_COEFFICIENT   ((ADC_READING_COEFFICIENT / 16.f) / 0.003f)     // convert ADC bits to Amps


typedef enum {
  CAN_ID_ESTOP              = 0x00U,
  CAN_ID_ID                 = 0x01U,
  CAN_ID_VERSION            = 0x02U,
  CAN_ID_HEARTBEAT          = 0x04U,
  CAN_ID_MODE_ERROR         = 0x06U,
  CAN_ID_FLASH              = 0x0FU,

  CAN_ID_FAST_FRAME_0       = 0x10U,
  CAN_ID_FAST_FRAME_1       = 0x11U,

  CAN_ID_ENCODER_CPR_OFFSET               = 0x20U,
  CAN_ID_ENCODER_FILTER                   = 0x21U,
  CAN_ID_ENCODER_POSITION_RAW_N_ROTATIONS = 0x22U,
  CAN_ID_POWERSTAGE_VOLTAGE_THRESHOLD     = 0x23U,
  CAN_ID_POWERSTAGE_FILTER                = 0x24U,
  CAN_ID_POWERSTAGE_BUS_VOLTAGE_MEASURED  = 0x25U,
  CAN_ID_MOTOR_POLE_PAIR_KV               = 0x26U,
  CAN_ID_MOTOR_PHASE_ORDER_FLUX_OFFSET    = 0x27U,

  CAN_ID_CURRENT_KP_KI                    = 0x30U,
  CAN_ID_CURRENT_LIMIT                    = 0x31U,
  CAN_ID_CURRENT_IA_IB_MEASURED           = 0x32U,
  CAN_ID_CURRENT_IC_MEASURED              = 0x33U,
  CAN_ID_CURRENT_VA_VB_SETPOINT           = 0x34U,
  CAN_ID_CURRENT_VC_SETPOINT              = 0x35U,
  CAN_ID_CURRENT_IALPHA_IBETA_MEASURED    = 0x36U,
  CAN_ID_CURRENT_VALPHA_VBETA_SETPOINT    = 0x37U,
  CAN_ID_CURRENT_VQ_VD_TARGET             = 0x38U,
  CAN_ID_CURRENT_VQ_VD_SETPOINT           = 0x39U,
  CAN_ID_CURRENT_IQ_ID_TARGET             = 0x3AU,
  CAN_ID_CURRENT_IQ_ID_MEASURED           = 0x3BU,
  CAN_ID_CURRENT_IQ_ID_SETPOINT           = 0x3CU,
  CAN_ID_CURRENT_IQ_ID_INTEGRATOR         = 0x3DU,

  CAN_ID_POSITION_KP_KI                   = 0x40U,
  CAN_ID_VELOCITY_KP_KI                   = 0x41U,
  CAN_ID_TORQUE_VELOCITY_LIMIT            = 0x42U,
  CAN_ID_POSITION_LIMIT                   = 0x43U,
  CAN_ID_TORQUE_TARGET                    = 0x44U,
  CAN_ID_TORQUE_MEASURED_SETPOINT         = 0x45U,
  CAN_ID_VELOCITY_TARGET                  = 0x46U,
  CAN_ID_VELOCITY_MEASURED_SETPOINT       = 0x47U,
  CAN_ID_POSITION_TARGET                  = 0x48U,
  CAN_ID_POSITION_MEASURED_SETPOINT       = 0x49U,
  CAN_ID_POSITION_VELOCITY_INTEGRATOR     = 0x4AU,

  CAN_ID_PING               = 0x7FU
} CAN_ID_Types;



typedef enum {
  // these are three safe modes
  MODE_DISABLED             = 0x00U,
  MODE_IDLE                 = 0x01U,

  // these are special modes
  MODE_DAMPING              = 0x02U,
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
