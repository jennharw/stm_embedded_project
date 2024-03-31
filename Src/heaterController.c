#include "heaterController.h"

static uint8_t m_state = 0;

void heaterControl(uint8_t onOff){

	  HAL_GPIO_WritePin(PB5_RE_AY_ON_OFF_CTRL_GPIO_Port, PB5_RE_AY_ON_OFF_CTRL_Pin, onOff);
	  m_state = onOff;
}

uint8_t getHeaterState(){
	return m_state;
}
