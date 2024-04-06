/*
 * oledController.h
 *
 *  Created on: Apr 6, 2024
 *      Author: choiharim
 */

#ifndef INC_OLEDCONTROLLER_H_
#define INC_OLEDCONTROLLER_H_
#include "ssd1306.h"
#include "defines.h"
#include "controlType.h"
#include "heaterController.h"
#include <stdio.h>

void opening();
void printDefault();
void printHeaterState(ON_OFF_t onOff);
void printTemper(int temper);
void startToggle();
void toggleScreen();
void printBackground();
void printTemperNoUpdate(int temper);

#endif /* INC_OLEDCONTROLLER_H_ */
