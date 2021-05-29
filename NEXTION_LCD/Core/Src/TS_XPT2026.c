

#include "main.h"
#include "stm32f0xx_hal.h"

extern uint16_t TS_KOR_X;
extern uint16_t TS_KOR_Y;
extern uint8_t tusa_basildi;

void Delay_Us()
{
	uint32_t iii=1000;
	while(iii)
	{
		iii--;
	}
}

//********************************************************
void TS_KOR_XY_OKU()
{
	while(!XPT2046(0xD0));

	 TS_KOR_Y=0;
	for (uint8_t j= 0; j < 10; ++j) {
		TS_KOR_Y+=XPT2046(0xD0);
	}

	 TS_KOR_X=0;
	for (uint8_t j= 0; j < 10; ++j) {
		TS_KOR_X+=XPT2046(0x90);
	}

 TS_KOR_Y=TS_KOR_Y/10; //1600 -160 320 tarafı
 TS_KOR_X=TS_KOR_X/10; //4080 -160 480 tarafı
}

//-*******************************************************
int XPT2046 (int ts_cmd)
{
	uint16_t xpt2046_rx_data=0;
    uint16_t i=0;
	HAL_GPIO_WritePin(TS_CS_GPIO_Port, TS_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TS_DCLCK_GPIO_Port, TS_DCLCK_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	// data gonderme kısmı
	for (i = 0; i < 8; ++i) {
		if(ts_cmd & 0x80)
			HAL_GPIO_WritePin(TS_DIN_GPIO_Port, TS_DIN_Pin, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(TS_DIN_GPIO_Port, TS_DIN_Pin, GPIO_PIN_RESET);

		ts_cmd=ts_cmd << 1;

		HAL_GPIO_WritePin(TS_DCLCK_GPIO_Port, TS_DCLCK_Pin, GPIO_PIN_RESET);
		Delay_Us();
		HAL_GPIO_WritePin(TS_DCLCK_GPIO_Port, TS_DCLCK_Pin, GPIO_PIN_SET);
		Delay_Us();
	}
//	HAL_GPIO_WritePin(TS_DIN_GPIO_Port, TS_DIN_Pin, GPIO_PIN_SET);

	HAL_Delay(1);
	// data okuma kısmı
	for (i = 0; i < 12; ++i) {
		HAL_GPIO_WritePin(TS_DCLCK_GPIO_Port, TS_DCLCK_Pin, GPIO_PIN_SET);
		Delay_Us();
		HAL_GPIO_WritePin(TS_DCLCK_GPIO_Port, TS_DCLCK_Pin, GPIO_PIN_RESET);
		Delay_Us();

		if(HAL_GPIO_ReadPin(TS_DOUT_GPIO_Port, TS_DOUT_Pin))xpt2046_rx_data++;
		xpt2046_rx_data=xpt2046_rx_data <<1;

	}

	HAL_GPIO_WritePin(TS_CS_GPIO_Port, TS_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(3);

	return xpt2046_rx_data;
}
