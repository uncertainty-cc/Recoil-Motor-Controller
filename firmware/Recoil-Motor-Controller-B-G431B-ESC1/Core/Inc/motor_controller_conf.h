/*
 * motor_controller_conf.h
 *
 *  Created on: Sep 8, 2022
 *      Author: TK
 */

#ifndef INC_MOTOR_CONTROLLER_CONF_H_
#define INC_MOTOR_CONTROLLER_CONF_H_

/** ======== Controller Settings ======== **/

/**
 * Firmware Version:
 * The firmware version is represented as a 32-bit hexadecimal number.
 * It follows the format: (MAJOR [7:4]) . (MINOR [3:2]) . (PATCH [1:0]).
 * For example, 0x00010005 represents version 1.0.5 of the firmware.
 */
//                                        YYYYmmdd
#define FIRMWARE_VERSION                0x20250120

/**
 * Device CAN ID:
 * This macro defines the CAN (Controller Area Network) ID of the device.
 * The CAN ID is a unique identifier for the device on the CAN bus.
 * The value should be set in range [1, 63].
 */
#define DEVICE_CAN_ID                   1

/**
 * First Time Bootup Flag:
 * If this is the first time the device is programmed, set this flag to 1 to configure Flash option byte.
 */
#define FIRST_TIME_BOOTUP               0             // Set to 1 for the first-time bootup routine, 0 for normal operation


/**
 * Load ID from Flash Flag:
 * This flag specifies whether the device should load the ID configuration from Flash memory.
 */
#define LOAD_ID_FROM_FLASH              1             // Set to 1 to load ID config from Flash, 0 to load default values

/**
 * Load Config from Flash Flag:
 * This flag specifies whether the device should load the configuration settings from Flash memory.
 * It excludes loading motor flux offset and CAN ID.
 */
#define LOAD_CONFIG_FROM_FLASH          1             // Set to 1 to load config settings (everything except
                                                      // motor flux offset and can id) from Flash, 0 to load default values
/**
 * Load Calibration from Flash Flag:
 * This flag indicates whether the device should load the encoder flux offset settings from Flash memory.
 */
#define LOAD_CALIBRATION_FROM_FLASH     1             // Set to 1 to load calibration settings, 0 to load default values


/**
 * Safety Watchdog Enabled Flag:
 * This flag controls whether the safety watchdog feature is enabled or disabled.
 * The safety watchdog is used for timeout monitoring.
 */
#define SAFETY_WATCHDOG_ENABLED         1             // Set to 1 to enable safety watchdog


/** ======== Encoder Configuration ======== **/
#define ENCODER_DIRECTION               +1
#define ENCODER_PRECISION_BITS          12

/** ======== Motor Selection ======== **/

#define MOTORPROFILE_MAD_M6C12_150KV
//#define MOTORPROFILE_MAD_5010_110KV
//#define MOTORPROFILE_MAD_5010_310KV
//#define MOTORPROFILE_MAD_5010_370KV

// phase order
#define MOTOR_PHASE_ORDER               +1

// nominal bus voltage (V)
#define NOMINAL_BUS_VOLTAGE             12.f


/** ======== Timing Configuration ======== **/
// current control loop frequency (Hz)
// = SYS_CLK / TIM_AAR / TIM_REPTITION = 160 MHz / 4000 / 2 = 20 kHz
#define COMMUTATION_FREQ                        20000.f

// position encoder update frequency (Hz), equals I2C packet rate
#define ENCODER_UPDATE_FREQ                     10000.f

// position control loop frequency (Hz)
#define POSITION_UPDATE_FREQ                    2000.f


/** ======== Constants ======== **/
// number of entries in the encoder offset table
#define ENCODER_LUT_ENTRIES             128

// (bits)
#define ADC_RESOLUTION                  4096          // 12-bit ADC

// (V / bits)
#define ADC_READING_COEFFICIENT         (3.3f / (float)ADC_RESOLUTION)

// convert ADC bits to Volts (V / bits)
#define ADC_BUS_VOLTAGE_COEFFICIENT     (ADC_READING_COEFFICIENT * ((18.f + 169.f) / 18.f))

// convert ADC bits to Amps (A / bits)
// = ((3V3 / ADC_RESOLUTION) / opamp_factor) / R
#define ADC_OPAMP_CURRENT_COEFFICIENT   ((ADC_READING_COEFFICIENT / 16.f) / 0.003f)


/** ======== Controller State Definitions ======== **/

/**
 * @brief Mode definition.
 */
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

/**
 * @brief ErrorCode definition.
 */
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
  ERROR_CAN_RX_FAULT              = 0b0000010000000000U,
  ERROR_CAN_TX_FAULT              = 0b0000100000000000U,
  ERROR_I2C_FAULT                 = 0b0001000000000000U,
} ErrorCode;

/** ======== CAN Packet Definitions ======== **/
/**
 * @brief CAN FrameFunction definition.
 */
typedef enum {
  FUNC_ESTOP                    = 0x00U,
  FUNC_INFO                     = 0x01U,
  FUNC_SAFETY_WATCHDOG          = 0x02U,
  FUNC_PING                     = 0x04U,
  FUNC_MODE                     = 0x05U,
  FUNC_FLASH                    = 0x0EU,
  FUNC_PARAM_READ               = 0x10U,
  FUNC_PARAM_WRITE              = 0x11U,
  FUNC_USR_FAST_FRAME_0         = 0x12U,
  FUNC_USR_FAST_FRAME_1         = 0x13U,
  FUNC_USR_FAST_FRAME_2         = 0x14U,
  FUNC_USR_FAST_FRAME_3         = 0x15U,
  FUNC_USR_FAST_FRAME_4         = 0x16U,
  FUNC_USR_FAST_FRAME_5         = 0x17U,
  FUNC_USR_FAST_FRAME_6         = 0x18U,
  FUNC_USR_FAST_FRAME_7         = 0x19U,
} FrameFunction;

/**
 * @brief CAN Parameter ID definition.
 */
typedef enum {
  PARAM_DEVICE_ID                                       = 0x000U,
  PARAM_FIRMWARE_VERSION                                = 0x004U,
  PARAM_WATCHDOG_TIMEOUT                                = 0x008U,
  PARAM_FAST_FRAME_FREQUENCY                            = 0x00CU,
  PARAM_MODE                                            = 0x010U,
  PARAM_ERROR                                           = 0x014U,
  PARAM_POSITION_CONTROLLER_UPDATE_COUNTER              = 0x018U,
  PARAM_POSITION_CONTROLLER_GEAR_RATIO                  = 0x01CU,
  PARAM_POSITION_CONTROLLER_POSITION_KP                 = 0x020U,
  PARAM_POSITION_CONTROLLER_POSITION_KI                 = 0x024U,
  PARAM_POSITION_CONTROLLER_VELOCITY_KP                 = 0x028U,
  PARAM_POSITION_CONTROLLER_VELOCITY_KI                 = 0x02CU,
  PARAM_POSITION_CONTROLLER_TORQUE_LIMIT                = 0x030U,
  PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT              = 0x034U,
  PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER        = 0x038U,
  PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER        = 0x03CU,
  PARAM_POSITION_CONTROLLER_POSITION_OFFSET             = 0x040U,
  PARAM_POSITION_CONTROLLER_TORQUE_TARGET               = 0x044U,
  PARAM_POSITION_CONTROLLER_TORQUE_MEASURED             = 0x048U,
  PARAM_POSITION_CONTROLLER_TORQUE_SETPOINT             = 0x04CU,
  PARAM_POSITION_CONTROLLER_VELOCITY_TARGET             = 0x050U,
  PARAM_POSITION_CONTROLLER_VELOCITY_MEASURED           = 0x054U,
  PARAM_POSITION_CONTROLLER_VELOCITY_SETPOINT           = 0x058U,
  PARAM_POSITION_CONTROLLER_POSITION_TARGET             = 0x05CU,
  PARAM_POSITION_CONTROLLER_POSITION_MEASURED           = 0x060U,
  PARAM_POSITION_CONTROLLER_POSITION_SETPOINT           = 0x064U,
  PARAM_POSITION_CONTROLLER_POSITION_INTEGRATOR         = 0x068U,
  PARAM_POSITION_CONTROLLER_VELOCITY_INTEGRATOR         = 0x06CU,
  PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA         = 0x070U,
  PARAM_CURRENT_CONTROLLER_I_LIMIT                      = 0x074U,
  PARAM_CURRENT_CONTROLLER_I_KP                         = 0x078U,
  PARAM_CURRENT_CONTROLLER_I_KI                         = 0x07CU,
  PARAM_CURRENT_CONTROLLER_I_A_MEASURED                 = 0x080U,
  PARAM_CURRENT_CONTROLLER_I_B_MEASURED                 = 0x084U,
  PARAM_CURRENT_CONTROLLER_I_C_MEASURED                 = 0x088U,
  PARAM_CURRENT_CONTROLLER_V_A_SETPOINT                 = 0x08CU,
  PARAM_CURRENT_CONTROLLER_V_B_SETPOINT                 = 0x090U,
  PARAM_CURRENT_CONTROLLER_V_C_SETPOINT                 = 0x094U,
  PARAM_CURRENT_CONTROLLER_I_ALPHA_MEASURED             = 0x098U,
  PARAM_CURRENT_CONTROLLER_I_BETA_MEASURED              = 0x09CU,
  PARAM_CURRENT_CONTROLLER_V_ALPHA_SETPOINT             = 0x0A0U,
  PARAM_CURRENT_CONTROLLER_V_BETA_SETPOINT              = 0x0A4U,
  PARAM_CURRENT_CONTROLLER_V_Q_TARGET                   = 0x0A8U,
  PARAM_CURRENT_CONTROLLER_V_D_TARGET                   = 0x0ACU,
  PARAM_CURRENT_CONTROLLER_V_Q_SETPOINT                 = 0x0B0U,
  PARAM_CURRENT_CONTROLLER_V_D_SETPOINT                 = 0x0B4U,
  PARAM_CURRENT_CONTROLLER_I_Q_TARGET                   = 0x0B8U,
  PARAM_CURRENT_CONTROLLER_I_D_TARGET                   = 0x0BCU,
  PARAM_CURRENT_CONTROLLER_I_Q_MEASURED                 = 0x0C0U,
  PARAM_CURRENT_CONTROLLER_I_D_MEASURED                 = 0x0C4U,
  PARAM_CURRENT_CONTROLLER_I_Q_SETPOINT                 = 0x0C8U,
  PARAM_CURRENT_CONTROLLER_I_D_SETPOINT                 = 0x0CCU,
  PARAM_CURRENT_CONTROLLER_I_Q_INTEGRATOR               = 0x0D0U,
  PARAM_CURRENT_CONTROLLER_I_D_INTEGRATOR               = 0x0D4U,
  PARAM_POWERSTAGE_HTIM                                 = 0x0D8U,
  PARAM_POWERSTAGE_HADC1                                = 0x0DCU,
  PARAM_POWERSTAGE_HADC2                                = 0x0E0U,
  PARAM_POWERSTAGE_ADC_READING_RAW                      = 0x0E4U,
  PARAM_POWERSTAGE_ADC_READING_OFFSET                   = 0x0ECU,
  PARAM_POWERSTAGE_UNDERVOLTAGE_THRESHOLD               = 0x0F4U,
  PARAM_POWERSTAGE_OVERVOLTAGE_THRESHOLD                = 0x0F8U,
  PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA             = 0x0FCU,
  PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED                 = 0x100U,
  PARAM_MOTOR_POLE_PAIRS                                = 0x104U,
  PARAM_MOTOR_KV_RATING                                 = 0x108U,
  PARAM_MOTOR_PHASE_ORDER                               = 0x10CU,
  PARAM_MOTOR_MAX_CALIBRATION_CURRENT                   = 0x110U,
  PARAM_ENCODER_HI2C                                    = 0x114U,
  PARAM_ENCODER_I2C_BUFFER                              = 0x118U,
  PARAM_ENCODER_I2C_UPDATE_COUNTER                      = 0x11CU,
  PARAM_ENCODER_CPR                                     = 0x120U,
  PARAM_ENCODER_POSITION_OFFSET                         = 0x124U,
  PARAM_ENCODER_VELOCITY_FILTER_ALPHA                   = 0x128U,
  PARAM_ENCODER_POSITION_RAW                            = 0x12CU,
  PARAM_ENCODER_N_ROTATIONS                             = 0x130U,
  PARAM_ENCODER_POSITION                                = 0x134U,
  PARAM_ENCODER_VELOCITY                                = 0x138U,
  PARAM_ENCODER_FLUX_OFFSET                             = 0x13CU,
  PARAM_ENCODER_FLUX_OFFSET_TABLE                       = 0x140U,
} Parameter;


#endif /* INC_MOTOR_CONTROLLER_CONF_H_ */
