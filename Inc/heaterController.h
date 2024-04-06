/*
 * heaterController.h
 *
 *  Created on: Mar 29, 2024
 *      Author: choiharim
 */

#ifndef SRC_HEATERCONTROLLER_H_
#define SRC_HEATERCONTROLLER_H_

#include "main.h"
#include "defines.h"
#include "controlType.h"
#include "ledController.h"
#include "oledController.h"

uint8_t getHeaterState();
void heaterControl(float temper);
void heaterOnOff(ON_OFF_t on_off);
void temper_up(void);
void temper_down(void);

void setFixedTemper(void);
int getFixedTemper(void);

#endif /* SRC_HEATERCONTROLLER_H_ */
