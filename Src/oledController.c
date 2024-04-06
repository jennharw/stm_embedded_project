#include "oledController.h"

static uint8_t m_toggle = 0;
static uint8_t m_toggle_count = 0;

void opening() {
	//  SSD1306_GotoXY (0,0);
	//  SSD1306_Puts ("HELLO", &Font_11x18, 1);
	//  SSD1306_GotoXY (10, 30);
	//  SSD1306_Puts ("  WORLD :)", &Font_11x18, 1);
	//  SSD1306_UpdateScreen(); //display
	//
	//  SSD1306_Clear();
	//  SSD1306_DrawBitmap(0,0,horse1,128,64,1);
	//  SSD1306_UpdateScreen();
	//  SSD1306_Clear();
	//  SSD1306_DrawBitmap(0,0,horse2,128,64,1);
	//  SSD1306_UpdateScreen();
	//  SSD1306_Clear();
	//  SSD1306_DrawBitmap(0,0,horse3,128,64,1);
	//  SSD1306_UpdateScreen();
	//  SSD1306_Clear();
	//  SSD1306_DrawBitmap(0,0,horse4,128,64,1);
	//  SSD1306_UpdateScreen();
	//
	//  HAL_Delay (2000);
	//  SSD1306_InvertDisplay(1);  // invert the display
	printDefault();
}

void printDefault() {

	SSD1306_InvertDisplay(0);
	SSD1306_Clear();
	SSD1306_GotoXY(1, 0);
	SSD1306_Puts("Temper Work", &Font_11x18, 1);
	SSD1306_GotoXY(0, 15);
	SSD1306_Puts("-----------", &Font_11x18, 1);
	SSD1306_GotoXY(14, 38);

	char temper_str[100] = "";
	itoa(DEFAULT_TEMPER, temper_str,10);
	strcat(temper_str,".0");
	SSD1306_Puts(temper_str, &Font_11x18, 1);
	SSD1306_GotoXY(81, 38);
	SSD1306_Puts("Off", &Font_11x18, 1);
	SSD1306_UpdateScreen();

}

void printHeaterState(ON_OFF_t onOff){
	SSD1306_GotoXY(81, 38);
	if(onOff == ON_t){
		SSD1306_Puts("On ", &Font_11x18, 1);
	}else{
		SSD1306_Puts("Off", &Font_11x18, 1);
	}
	SSD1306_UpdateScreen();
}

void printTemper(int temper) {

	SSD1306_GotoXY(14, 38);
	char temper_str[100] = "";
	sprintf(temper_str, "%2d.0", temper);
	SSD1306_Puts(temper_str, &Font_11x18, 1);
	SSD1306_UpdateScreen();
}

void startToggle() {
	if(m_toggle_count == 0 ){
		m_toggle_count = 4;
	}
}

void toggleScreen() {

	if (m_toggle_count > 0) {
		if (!m_toggle) {
			SSD1306_Clear();
			SSD1306_UpdateScreen();
			m_toggle = 1;
		} else {
			printBackground();
			printTemperNoUpdate(getFixedTemper());
			printHeaterState(OFF_t);

			m_toggle = 0;
		}
	}
	if(m_toggle_count > 0 ){
		m_toggle_count--;
	}
}

void printBackground() {

	SSD1306_InvertDisplay(0);
	SSD1306_Clear();
	SSD1306_GotoXY(1, 0);
	SSD1306_Puts("Temper Work", &Font_11x18, 1);
	SSD1306_GotoXY(0, 15);
	SSD1306_Puts("-----------", &Font_11x18, 1);

}

void printTemperNoUpdate(int temper) {

	SSD1306_GotoXY(14, 38);
	char temper_str[100] = "";
	sprintf(temper_str, "%2d.0", temper);
	SSD1306_Puts(temper_str, &Font_11x18, 1);

}


