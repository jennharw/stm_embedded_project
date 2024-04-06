#include "heaterController.h"

static uint8_t m_state = 0;

static int m_desired_temper = DEFAULT_TEMPER;
static int m_fixed_temper = DEFAULT_TEMPER;

void heaterControl(float temper){


  if(m_state){
	  if((int)temper >= (m_fixed_temper - GOING_UP_GAP)){
		heaterOnOff(OFF_t);
	}
  }else{
	if((int)temper <  m_fixed_temper - GOING_DOWN_GAP){
		heaterOnOff(ON_t);
	}
  }
}

uint8_t getHeaterState(){
	return m_state;
}


void heaterOnOff(ON_OFF_t onOff){
	HAL_GPIO_WritePin(PB5_RE_AY_ON_OFF_CTRL_GPIO_Port, PB5_RE_AY_ON_OFF_CTRL_Pin, onOff);

	if(onOff == ON_t){
		led2OnOff(ON_t);
		printHeaterState(OFF_t);
	}else{
		led2OnOff(ON_t);
		printHeaterState(OFF_t);
	}
}

void temper_up(){

	m_desired_temper++;
	if(m_desired_temper > 99){
		m_desired_temper = 0;
	}
	printTemper(m_desired_temper);
}

void temper_down(){

	m_desired_temper--;
	if(m_desired_temper < 0){
		m_desired_temper = 99;
	}
	printTemper(m_desired_temper);

}

int getFixedTemper(void){
	return m_desired_temper;
}



void setFixedTemper(){
	m_fixed_temper = m_desired_temper;
}
