/*
 * app.h
 *
 *  Created on: Aug 24, 2022
 *      Author: TK
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <string.h>
#include <stdio.h>

#include "stm32g4xx_hal.h"

#include "motor_controller_conf.h"
#include "main.h"
#include "can.h"
#include "motor_controller.h"


uint8_t APP_getUserButton();

float APP_getUserPot();

void APP_initFlashOption();

void APP_init();

void APP_main();

#endif /* INC_APP_H_ */
