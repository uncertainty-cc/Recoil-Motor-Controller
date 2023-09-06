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

#include "motor_controller.h"
#include "main.h"

/**
 * @brief Initialize Flash Option Bytes to always boot from Flash.
 *
 * @note The device need to be power-cycled to apply the changes.
 */
void APP_initFlashOption();

/**
 * @brief Get the state of the button on the daughter board.
 *
 * @return The state of the user button:
 *         - 0 if the button is not pressed.
 *         - 1 if the button is pressed.
 */
uint8_t APP_getUserButton();

/**
 * @brief Get the value of the potentiometer on the daughter board.
 *
 * @return Scaled value of the user potentiometer reading in range [0.0, 1.0]
 */
float APP_getUserPot();

void APP_init();

void APP_main();

#endif /* INC_APP_H_ */
