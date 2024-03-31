/*
 * heaterController.h
 *
 *  Created on: Mar 29, 2024
 *      Author: choiharim
 */

#ifndef SRC_HEATERCONTROLLER_H_
#define SRC_HEATERCONTROLLER_H_

#include "main.h"
enum{
	t_OFF = 0,
	t_ON = 1
};

uint8_t getHeaterState();
void heaterControl(uint8_t onOff);


#endif /* SRC_HEATERCONTROLLER_H_ */
