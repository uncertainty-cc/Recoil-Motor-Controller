/*
 * motor_profiles.h
 *
 *  Created on: Sep 5, 2023
 *      Author: tk
 */

#ifndef INC_MOTOR_PROFILES_H_
#define INC_MOTOR_PROFILES_H_

#include "motor_controller_conf.h"


/** ======== Motor Profile Definitions ======== **/

#ifdef MOTORPROFILE_MAD_M6C12_150KV
#define MOTOR_POLE_PAIRS                      14
#define MOTOR_TORQUE_CONSTANT                 0.0919f
#define MOTOR_PHASE_RESISTANCE                0.1886f
#define MOTOR_PHASE_INDUCTANCE                0.0000325f
#define MOTOR_CALIBRATION_CURRENT             5
#endif

#ifdef MOTORPROFILE_MAD_5010_110KV
#define MOTOR_POLE_PAIRS                      14
#define MOTOR_TORQUE_CONSTANT                 0.1176f
#define MOTOR_PHASE_RESISTANCE                0.6193f
#define MOTOR_PHASE_INDUCTANCE                0.0000850f
#define MOTOR_CALIBRATION_CURRENT             3
#endif

#ifdef MOTORPROFILE_MAD_5010_310KV
#define MOTOR_POLE_PAIRS                      14
//#define MOTOR_TORQUE_CONSTANT                 370
#define MOTOR_PHASE_RESISTANCE                0.05735062549544696f
#define MOTOR_PHASE_INDUCTANCE                3.325681588015225e-05f
#define MOTOR_CALIBRATION_CURRENT             5
#endif

#ifdef MOTORPROFILE_MAD_5010_370KV
#define MOTOR_POLE_PAIRS                      14
//#define MOTOR_TORQUE_CONSTANT                 370
#define MOTOR_PHASE_RESISTANCE                0.03000304860153151f
#define MOTOR_PHASE_INDUCTANCE                1.0717319302328058e-05f
#define MOTOR_CALIBRATION_CURRENT             5
#endif


#endif /* INC_MOTOR_PROFILES_H_ */
