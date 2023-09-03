# Recoil Motor Controller

## Overview

A brushless DC motor controller with DRV8350RS gate driver, three-phase current sampling, and SPI absolute position encoder implementing FOC torque control.


- 3 phase brushless DC motor controller 
- input voltage: 12 - 50 V
- maximum current: 10 A without heat sink, 50 A with heat sink
- temperature: 0 - 70 ℃
- PWM frequency: 40 kHz
- current loop frequency: 20 kHz
- PID position and velocity loop frequency: 4 kHz
- communication: CAN 2.0

> **Warning**
> These files are still under development and undergoing testing. While we do
> aspire to produce a design that others can easily fabricate, we do not yet
> recommend making them for yourself! 


## First-time power up settings

#### Update Flash option bytes

Go to `motor_controller_conf.h` and set `FIRST_TIME_BOOTUP` to **1**. Upload the program to the board, and then power-cycle the board. 

The MCU should configure the Flash option bytes to boot from Flash regardless of the BOOTx pin mode.

Set `FIRST_TIME_BOOTUP` to **0** and reflash the program.


#### Update Flash user settings

Set `LOAD_ID_FROM_FLASH`, `LOAD_CONFIG_FROM_FLASH`, `LOAD_CALIBRATION_FROM_FLASH`, and `SAFETY_WATCHDOG_ENABLED` to **0**. Upload the program to the board to update contents in the user settings section in Flash. Selet the correct motor type in "Motor Selection" section.

Then, set `LOAD_ID_FROM_FLASH`, `LOAD_CONFIG_FROM_FLASH`, and `LOAD_CALIBRATION_FROM_FLASH`. Run calibration to get the correct calibration data.

For deployment, set `SAFETY_WATCHDOG_ENABLED` to **1**.


### Motor Mode Mapping

#### `0x00 - MODE_DISABLED`

The controller is disabled.

#### `0x01 - MODE_IDLE`

The motor is disabled.

#### `0x02 - MODE_DAMPING`

All low side MOSFETs are opened, and the motor phases are shorted to ground. The motor have high impedance. 

> It's a good idea to switch to this mode before switching back to idle mode to avoid sudden collapse of the robot.

#### `0x05 - MODE_CALIBRATION`

The motor performs self-calibration.

#### `0x10 - MODE_CURRENT`

Closed-loop current control. User sets `controller.current_controller.i_q_target` and `controller.current_controller.i_d_target`.

#### `0x11 - MODE_TORQUE`

Closed-loop torque control. User sets `controller->torque_target`.

#### `0x12 - MODE_VELOCITY`

Closed-loop velocity control. User sets `controller->velocity_target`.

#### `0x13 - MODE_POSITION`

Closed-loop position control. User sets `controller->position_target`.

#### `0x20 - MODE_VABC_OVERRIDE`

Open-loop phase voltage control. User sets `controller->v_a_setpoint`, `controller->v_b_setpoint`, and `controller->v_c_setpoint`.

#### `0x21 - MODE_VALPHABETA_OVERRIDE`

Open-loop alpha-beta frame voltage control. User sets `controller->v_alpha_setpoint` and `controller->v_beta_setpoint`.

#### `0x22 - MODE_VQD_OVERRIDE`

Open-loop d-q frame voltage control. User sets `controller->v_d_setpoint` and `controller->v_q_setpoint`.

#### `0x80 - MODE_DEBUG`

For debugging.


### Error Code Mapping

TBD. Still need to be finalized.

### CAN ID Mapping

#### `0x00 - CAN_ID_ESTOP`

Emergency stop.

#### `0x01 - CAN_ID_INFO`

contains device id and version

device_id = data[7:0]

firmware_version = data[63:32]

#### `0x02 - CAN_ID_SAFETY_WATCHDOG`

safety watchdog heartbeat message.

#### `0x05 - CAN_ID_MODE`

#### `0x0E - CAN_ID_FLASH`

#### `0x10 - CAN_ID_USR_PARAM_READ`

<!--
[value, setting_idx]

| setting field               | setting_idx | setting_idx (hex) |
| --------------------------- | ----------- | ----------------- |
| CMD_ENCODER_CPR             | 16  | 0x10              |
| CMD_ENCODER_OFFSET          | 17  | 0x11              |
| CMD_ENCODER_FILTER          | 18  | 0x12              |
| CMD_ENCODER_FLUX_OFFSET     | 19  | 0x13           |
| CMD_ENCODER_POSITION_RAW    | 20  | 0x14              |
| CMD_ENCODER_N_ROTATIONS     | 21  | 0x15              |
| CMD_POWERSTAGE_VOLTAGE_THRESHOLD_LOW      | 5  | 0x16              |
| CMD_POWERSTAGE_VOLTAGE_THRESHOLD_HIGH     | 6  | 0x17              |
| CMD_POWERSTAGE_FILTER                     | 7  | 0x18              |
| CMD_POWERSTAGE_BUS_VOLTAGE_MEASURED       | 8  | 0x19              |
| CMD_MOTOR_POLE_PAIR         | 9  | 0x19              |
| CMD_MOTOR_KV                | 10 | 0x1A              |
| CMD_MOTOR_PHASE_ORDER       | 11 | 0x1B              |
| CMD_MOTOR_FLUX_OFFSET       | 12 | 0x1C              |
| CMD_CURRENT_KP              | 13 | 0x1D              |
| CMD_CURRENT_KI              | 14 | 0x1E              |
| CMD_CURRENT_LIMIT           | 15 | 0x1F              |
| CMD_CURRENT_IA_MEASURED     | 16 | 0x10              |
| CMD_CURRENT_IB_MEASURED     | 17 | 0x11              |
| CMD_CURRENT_IC_MEASURED     | 18 | 0x12              |
| CMD_CURRENT_VA_SETPOINT     | 19 | 0x13              |
| CMD_CURRENT_VB_SETPOINT     | 20 | 0x14              |
| CMD_CURRENT_VC_SETPOINT     | 21 | 0x15              |
| CMD_CURRENT_IALPHA_MEASURED | 22 | 0x16              |
| CMD_CURRENT_IBETA_MEASURED  | 23 | 0x17              |
| CMD_CURRENT_VALPHA_SETPOINT | 24 | 0x18              |
| CMD_CURRENT_VBETA_SETPOINT  | 25 | 0x19              |
| CMD_CURRENT_VQ_TARGET       | 26 | 0x1A              |
| CMD_CURRENT_VD_TARGET       | 27 | 0x1B              |
| CMD_CURRENT_VQ_SETPOINT     | 28 | 0x1C              |
| CMD_CURRENT_VD_SETPOINT     | 29 | 0x1D              |
| CMD_CURRENT_IQ_TARGET       | 30 | 0x1E              |
| CMD_CURRENT_ID_TARGET       | 31 | 0x1F              |
| CMD_CURRENT_IQ_MEASURED     | 32 | 0x20              |
| CMD_CURRENT_ID_MEASURED     | 33 | 0x21              |
| CMD_CURRENT_IQ_SETPOINT     | 34 | 0x22              |
| CMD_CURRENT_ID_SETPOINT     | 35 | 0x23              |
| CMD_CURRENT_IQ_INTEGRATOR   | 36 | 0x24              |
| CMD_CURRENT_ID_INTEGRATOR   | 37 | 0x25              |
| CMD_POSITION_KP             | 38 | 0x26              |
| CMD_POSITION_KI             | 39 | 0x27              |
| CMD_VELOCITY_KP             | 40 | 0x28              |
| CMD_VELOCITY_KI             | 41 | 0x29              |
| CMD_TORQUE_LIMIT            | 42 | 0x2A              |
| CMD_VELOCITY_LIMIT          | 43 | 0x2B              |
| CMD_POSITION_LIMIT_LOW      | 44 | 0x2C              |
| CMD_POSITION_LIMIT_HIGH     | 45 | 0x2D              |
| CMD_TORQUE_TARGET           | 46 | 0x2E              |
| CMD_TORQUE_MEASURED         | 47 | 0x2F              |
| CMD_TORQUE_SETPOINT         | 48 | 0x30              |
| CMD_VELOCITY_TARGET         | 49 | 0x31              |
| CMD_VELOCITY_MEASURED       | 50 | 0x32              |
| CMD_VELOCITY_SETPOINT       | 51 | 0x33              |
| CMD_POSITION_TARGET         | 52 | 0x34              |
| CMD_POSITION_MEASURED       | 53 | 0x35              |
| CMD_POSITION_SETPOINT       | 54 | 0x36              |
| CMD_VELOCITY_INTEGRATOR     | 55 | 0x37              |
| CMD_POSITION_INTEGRATOR     | 56 | 0x38              |

-->

#### `0x11 - CAN_ID_USR_PARAM_WRITE`

#### `0x12 - CAN_ID_USR_FAST_FRAME_0`

write: [torque_limit, position_target]

read: [torque_measured, position_measured]

#### `0x13 - CAN_ID_USR_FAST_FRAME_1`

write: [position_ki, position_kp]

read: [0, velocity_measured]

#### `0x14 - CAN_ID_USR_DEBUG_0`

#### `0x15 - CAN_ID_USR_DEBUG_1`

#### `0x16 - CAN_ID_USR_DEBUG_2`

#### `0x1F - CAN_ID_PING`

