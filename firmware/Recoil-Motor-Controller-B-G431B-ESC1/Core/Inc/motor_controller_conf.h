/*
 * motor_controller_conf.h
 *
 *  Created on: Sep 8, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_CONTROLLER_CONF_H_
#define INC_MOTOR_CONTROLLER_CONF_H_

/** ======== Controller Settings ======== **/
// (MAJOR [7:4]) . (MINOR [3:2]) . (PATCH [1:0])
#define FIRMWARE_VERSION                0x00010003

// min 1, max 63
#define DEVICE_CAN_ID                   1

#define FIRST_TIME_BOOTUP               0             // first time bootup: change Flash option byte, store config to Flash

#define LOAD_ID_FROM_FLASH              1             // load ID config from Flash
#define LOAD_CONFIG_FROM_FLASH          0             // load config settings from Flash (everything except motor flux offset and can id)
#define LOAD_CALIBRATION_FROM_FLASH     1             // load encoder flux offset settings from Flash
#define SAFETY_WATCHDOG_ENABLED         0             // timeout


/** ======== Motor Selection ======== **/

//#define MOTORPROFILE_MAD_M6C12_150KV
//#define MOTORPROFILE_MAD_5010_110KV
//#define MOTORPROFILE_MAD_5010_310KV
#define MOTORPROFILE_MAD_5010_370KV

/** ======== Motor Calibration Phase Current Configuration ======== **/
#ifdef MOTORPROFILE_MAD_M6C12_150KV
#define CALIBRATION_CURRENT             5
#endif
#ifdef MOTORPROFILE_MAD_5010_110KV
#define CALIBRATION_CURRENT             3
#endif
#ifdef MOTORPROFILE_MAD_5010_310KV
#define CALIBRATION_CURRENT             5
#endif
#ifdef MOTORPROFILE_MAD_5010_370KV
#define CALIBRATION_CURRENT             5
#endif


/** ======== Timing Configuration ======== **/

// current control loop frequency (Hz)
// = SYS_CLK / TIM_AAR / TIM_REPTITION = 160 MHz / 4000 / 2 = 20 kHz
#define COMMUTATION_FREQ                20000

// position encoder update frequency (Hz), equals I2C packet rate
#define ENCODER_UPDATE_FREQ             10000

// position control loop frequency (Hz)
#define POSITION_UPDATE_FREQ            2000

// current PI loop gain cutoff frequency (Hz)
#define CURRENT_LOOP_BANDWIDTH          1000

// encoder filter loop gain cutoff frequency (Hz)
#define ENCODER_FILTER_BANDWIDTH        2000


/** ======== Constants ======== **/

// number of entries in the encoder offset table
#define ENCODER_LUT_RESOLUTION          128

// (bits)
#define ADC_RESOLUTION                  4096          // 12-bit ADC

// (V / bits)
#define ADC_READING_COEFFICIENT         (3.3f / (float)ADC_RESOLUTION)

// (V / bits)
#define ADC_BUS_VOLTAGE_COEFFICIENT     (ADC_READING_COEFFICIENT * ((18.f + 169.f) / 18.f))   // convert ADC bits to Volts

// (A / bits)                                 // = ((3V3 / ADC_RESOLUTION) / opamp_factor) / R
#define ADC_OPAMP_CURRENT_COEFFICIENT   ((ADC_READING_COEFFICIENT / 16.f) / 0.003f)     // convert ADC bits to Amps


/** ======== Controller State Definitions ======== **/
typedef enum {
  // these are three safe modes
  MODE_DISABLED                   = 0x00U,
  MODE_IDLE                       = 0x01U,

  // these are special modes
  MODE_DAMPING                    = 0x02U,
  MODE_CALIBRATION                = 0x05U,

  // these are closed-loop modes
  MODE_CURRENT                    = 0x10U,
  MODE_TORQUE                     = 0x11U,
  MODE_VELOCITY                   = 0x12U,
  MODE_POSITION                   = 0x13U,

  // these are open-loop modes
  MODE_VABC_OVERRIDE              = 0x20U,
  MODE_VALPHABETA_OVERRIDE        = 0x21U,
  MODE_VQD_OVERRIDE               = 0x22U,

  MODE_DEBUG                      = 0x80U,
} Mode;

typedef enum {
  ERROR_NO_ERROR                  = 0b0000000000000000U,
  ERROR_GENERAL                   = 0b0000000000000001U,
  ERROR_ESTOP                     = 0b0000000000000010U,
  ERROR_INITIALIZATION_ERROR      = 0b0000000000000100U,
  ERROR_CALIBRATION_ERROR         = 0b0000000000001000U,
  ERROR_POWERSTAGE_ERROR          = 0b0000000000010000U,
  ERROR_INVALID_MODE              = 0b0000000000100000U,
  ERROR_WATCHDOG_TIMEOUT          = 0b0000000001000000U,
  ERROR_OVER_VOLTAGE              = 0b0000000010000000U,
  ERROR_OVER_CURRENT              = 0b0000000100000000U,
  ERROR_OVER_TEMPERATURE          = 0b0000001000000000U,
  ERROR_CAN_TX_FAULT              = 0b0000010000000000U,
  ERROR_I2C_FAULT                 = 0b0000100000000000U,
} ErrorCode;


/** ======== CAN Packet Definitions ======== **/
typedef enum {
  CAN_ID_ESTOP                    = 0x00U,
  CAN_ID_INFO                     = 0x01U,
  CAN_ID_SAFETY_WATCHDOG          = 0x02U,
  CAN_ID_MODE                     = 0x05U,
  CAN_ID_FLASH                    = 0x0EU,

  CAN_ID_USR_PARAM_READ           = 0x10U,
  CAN_ID_USR_PARAM_WRITE          = 0x11U,
  CAN_ID_USR_FAST_FRAME_0         = 0x12U,
  CAN_ID_USR_FAST_FRAME_1         = 0x13U,
  CAN_ID_USR_DEBUG_0              = 0x14U,
  CAN_ID_USR_DEBUG_1              = 0x15U,
  CAN_ID_USR_DEBUG_2              = 0x16U,

  CAN_ID_PING                     = 0x1FU
} CAN_ID_Types;

typedef enum {
  CMD_ENCODER_CPR                 = 0x10U,
  CMD_ENCODER_OFFSET              = 0x11U,
  CMD_ENCODER_FILTER_BANDWIDTH    = 0x12U,
  CMD_ENCODER_FLUX_OFFSET         = 0x13U,
  CMD_ENCODER_POSITION_RAW        = 0x14U,
  CMD_ENCODER_N_ROTATIONS         = 0x15U,
  CMD_POWERSTAGE_VOLTAGE_THRESHOLD_LOW    = 0x16U,
  CMD_POWERSTAGE_VOLTAGE_THRESHOLD_HIGH   = 0x17U,
  CMD_POWERSTAGE_FILTER                   = 0x18U,
  CMD_POWERSTAGE_BUS_VOLTAGE_MEASURED     = 0x19U,
  CMD_MOTOR_POLE_PAIR             = 0x1AU,
  CMD_MOTOR_KV                    = 0x1BU,
  CMD_MOTOR_PHASE_ORDER           = 0x1CU,
  CMD_MOTOR_PHASE_RESISTANCE      = 0x1DU,
  CMD_MOTOR_PHASE_INDUCTANCE      = 0x1EU,
  CMD_CURRENT_BANDWIDTH           = 0x1FU,
  CMD_CURRENT_LIMIT               = 0x20U,
  CMD_CURRENT_KP                  = 0x21U,
  CMD_CURRENT_KI                  = 0x22U,
  CMD_CURRENT_IA_MEASURED         = 0x23U,
  CMD_CURRENT_IB_MEASURED         = 0x24U,
  CMD_CURRENT_IC_MEASURED         = 0x25U,
  CMD_CURRENT_VA_SETPOINT         = 0x26U,
  CMD_CURRENT_VB_SETPOINT         = 0x27U,
  CMD_CURRENT_VC_SETPOINT         = 0x28U,
  CMD_CURRENT_IALPHA_MEASURED     = 0x29U,
  CMD_CURRENT_IBETA_MEASURED      = 0x2AU,
  CMD_CURRENT_VALPHA_SETPOINT     = 0x2BU,
  CMD_CURRENT_VBETA_SETPOINT      = 0x2CU,
  CMD_CURRENT_VQ_TARGET           = 0x2DU,
  CMD_CURRENT_VD_TARGET           = 0x2EU,
  CMD_CURRENT_VQ_SETPOINT         = 0x2FU,
  CMD_CURRENT_VD_SETPOINT         = 0x30U,
  CMD_CURRENT_IQ_TARGET           = 0x31U,
  CMD_CURRENT_ID_TARGET           = 0x32U,
  CMD_CURRENT_IQ_MEASURED         = 0x33U,
  CMD_CURRENT_ID_MEASURED         = 0x34U,
  CMD_CURRENT_IQ_SETPOINT         = 0x35U,
  CMD_CURRENT_ID_SETPOINT         = 0x36U,
  CMD_CURRENT_IQ_INTEGRATOR       = 0x37U,
  CMD_CURRENT_ID_INTEGRATOR       = 0x38U,
  CMD_POSITION_KP                 = 0x39U,
  CMD_POSITION_KI                 = 0x3AU,
  CMD_VELOCITY_KP                 = 0x3BU,
  CMD_VELOCITY_KI                 = 0x3CU,
  CMD_TORQUE_LIMIT                = 0x3DU,
  CMD_VELOCITY_LIMIT              = 0x3EU,
  CMD_POSITION_LIMIT_LOW          = 0x3FU,
  CMD_POSITION_LIMIT_HIGH         = 0x40U,
  CMD_TORQUE_TARGET               = 0x41U,
  CMD_TORQUE_MEASURED             = 0x42U,
  CMD_TORQUE_SETPOINT             = 0x43U,
  CMD_VELOCITY_TARGET             = 0x44U,
  CMD_VELOCITY_MEASURED           = 0x45U,
  CMD_VELOCITY_SETPOINT           = 0x46U,
  CMD_POSITION_TARGET             = 0x47U,
  CMD_POSITION_MEASURED           = 0x48U,
  CMD_POSITION_SETPOINT           = 0x49U,
  CMD_VELOCITY_INTEGRATOR         = 0x4AU,
  CMD_POSITION_INTEGRATOR         = 0x4BU,
} Command;


#endif /* INC_MOTOR_CONTROLLER_CONF_H_ */
