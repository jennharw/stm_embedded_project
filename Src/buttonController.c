#include "buttonController.h"

void checkButton()
{
	if(g_f_sw_up){
	//		  printf("push g_f_sw_up\r\n");
			  g_f_sw_up=0;
			  temper_up();
		  }
		  if(g_f_sw_down){
//			  printf("push g_f_sw_down\r\n");
			  g_f_sw_down=0;
			  temper_down();
		  }

		  if(g_f_sw_fix){
//			  printf("push g_f_sw_fix\r\n");
			  g_f_sw_fix=0;
			  startToggle();
			  setFixedTemper();
			  //HAL_GPIO_TogglePin(PB5_RELAY_ON_OFF_CTRL_GPIO_Port, PB5_RELAY_ON_OFF_CTRL_Pin);
		  }
		  if(g_f_sw_on){
//			  printf("push g_f_sw_on\r\n");
			  g_f_sw_on=0;
		  }
}

ON_OFF_t geSWState(){
	if(HAL_GPIO_ReadPin(PB12_START_SW_PIN_GPIO_Port, PB12_START_SW_PIN_Pin)){
			return OFF_t;
	}else{
			return ON_t;
	}
}
