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

### Function ID Mapping

#### `0x00` ESTOP

CAN_ID_ESTOP

#### `0x01` INFO

contains device id and version

device_id = data[7:0]

firmware_version = data[63:32]

#### `0x03` SAFETY_WATCHDOG

safety watchdog heartbeat message

#### `0x06` MODE

#### `0x0F` FLASH

#### `0x10` USR_SETTING_READ

[value, setting_idx]

| setting field | setting_idx  |
| ------------- | ------------ |
| ENCODER_CPR             | 0  |
| ENCODER_OFFSET          | 1  |
| ENCODER_FILTER          | 2  |
| ENCODER_POSITION_RAW    | 3  |
| ENCODER_N_ROTATIONS     | 4  |
| POWERSTAGE_VOLTAGE_THRESHOLD_LOW | 5  |
| POWERSTAGE_VOLTAGE_THRESHOLD_HIGH | 6  |
| POWERSTAGE_FILTER       | 7  |
| POWERSTAGE_BUS_VOLTAGE_MEASURED | 8  |
| MOTOR_POLE_PAIR         | 9  |
| MOTOR_KV                | 10 |
| MOTOR_PHASE_ORDER       | 11 |
| MOTOR_FLUX_OFFSET       | 12 |
| CURRENT_KP              | 13 |
| CURRENT_KI              | 14 |
| CURRENT_LIMIT           | 15 |
| CURRENT_IA_MEASURED     | 16 |
| CURRENT_IB_MEASURED     | 17 |
| CURRENT_IC_MEASURED     | 18 |
| CURRENT_VA_SETPOINT     | 19 |
| CURRENT_VB_SETPOINT     | 20 |
| CURRENT_VC_SETPOINT     | 21 |
| CURRENT_IALPHA_MEASURED | 22 |
| CURRENT_IBETA_MEASURED  | 23 |
| CURRENT_VALPHA_SETPOINT | 24 |
| CURRENT_VBETA_SETPOINT  | 25 |
| CURRENT_VQ_TARGET       | 26 |
| CURRENT_VD_TARGET       | 27 |
| CURRENT_VQ_SETPOINT     | 28 |
| CURRENT_VD_SETPOINT     | 29 |
| CURRENT_IQ_TARGET       | 30 |
| CURRENT_ID_TARGET       | 31 |
| CURRENT_IQ_MEASURED     | 32 |
| CURRENT_ID_MEASURED     | 33 |
| CURRENT_IQ_SETPOINT     | 34 |
| CURRENT_ID_SETPOINT     | 35 |
| CURRENT_IQ_INTEGRATOR   | 36 |
| CURRENT_ID_INTEGRATOR   | 37 |
| POSITION_KP             | 38 |
| POSITION_KI             | 39 |
| VELOCITY_KP             | 40 |
| VELOCITY_KI             | 41 |
| TORQUE_LIMIT            | 42 |
| VELOCITY_LIMIT          | 43 |
| POSITION_LIMIT_LOW      | 44 |
| POSITION_LIMIT_HIGH     | 45 |
| TORQUE_TARGET           | 46 |
| TORQUE_MEASURED         | 47 |
| TORQUE_SETPOINT         | 48 |
| VELOCITY_TARGET         | 49 |
| VELOCITY_MEASURED       | 50 |
| VELOCITY_SETPOINT       | 51 |
| POSITION_TARGET         | 52 |
| POSITION_MEASURED       | 53 |
| POSITION_SETPOINT       | 54 |
| VELOCITY_INTEGRATOR     | 55 |
| POSITION_INTEGRATOR     | 56 |

#### `0x11` USR_SETTING_WRITE

#### `0x12` USR_FAST_FRAME_0

write: [torque_limit, position_target]

read: [torque_measured, position_measured]

#### `0x13` USR_FAST_FRAME_1

write: [position_ki, position_kp]

read: [0, velocity_measured]

#### `0x14` USR_DEBUG_0

#### `0x15` USR_DEBUG_1

#### `0x16` USR_DEBUG_2

#### `0x1F` PING

