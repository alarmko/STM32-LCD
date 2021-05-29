/* Includes ------------------------------------------------------------------*/
#include "ILI9488.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

extern char buffer[1024]; // to store data
extern uint8_t sn;


#define Write_memory_start         0x2c
// parameters_q-ty   destination_adr values...
 static uint8_t const  ManufacturerCommandAccessProtect1[] =      {1,0xB0,0x1E};
 static uint8_t const  ManufacturerCommandAccessProtect2[] =      {1,0xB0,0x00};
 static uint8_t const  MemoryAccessInterfaceSetting[] =           {4,0xB3,0x02,0x00,0x00,0x10};
 static uint8_t const  DisplayModeFrameMemoryWriteModeSetting[] = {1,0xB4,0x00};
 static uint8_t const  PanelDrivingSetting[] =                    {8,0xC0,0x03,0x3B,0x00,0x00,0x00,0x01,0x00,0x43};
 static uint8_t const  DisplayTimingSettingNormalMode[] =         {4,0xC1,0x08,0x15,0x08,0x08};
 static uint8_t const  VCOMsetting[] =                            {4,0xC4,0x15,0x03,0x03,0x01};
 static uint8_t const  InterfaceSetting[] =                       {1,0xC6,0x02};
 static uint8_t const  GammaSet[] =                               {10,0xC8,0x0C,0x05,0x0A,0x6B,0x04,0x06,0x15,0x10,0x00,0x60};
 static uint8_t const  Set_address_mode[] =                        {1,0x36,0b00101010};
 static uint8_t const  Set_pixel_format[] =                       {1,0x3A,0x55};
 static uint8_t const  Exit_idle_mode[] =                         {0,0x38};
 static uint8_t const  PowerSetting[] =                           {4,0xD0,0x07,0x07,0x14,0xA2};
 static uint8_t const  VCOMsetting2[] =                           {3,0xD1,0x03,0x5A,0x10};
 static uint8_t const  PowerSettingNormalMode[] =                 {3,0xD2,0x03,0x04,0x04};
 static uint8_t const  Soft_reset[] =                             {0,0x11};
 static uint8_t const  Set_column_addressR[] =                    {4,0x2A,0x00,0x00, (uint8_t)(LCD_ILI9488_WidthX>>8), (uint8_t)(LCD_ILI9488_WidthX & 0x00ff)};
 static uint8_t const  Set_page_addressR[] =                      {4,0x2B,0x00,0x00, (uint8_t)(LCD_ILI9488_WidthY>>8), (uint8_t)(LCD_ILI9488_WidthY & 0x00ff)};
 static uint8_t const  Set_display_off[] =                        {0,0x28};
 static uint8_t const  Set_display_on[] =                         {0,0x29};

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//well routed bus needs no extra delays. in other cases, enable them
#define bus_delay_function()
//#define bus_delay_function() delay_cycles_irq_enabled(10)
//#define bus_delay_function() delay_us(10)


static inline __attribute__((optimize("Ofast"))) void WritePulse (void){
//	bus_delay_function();
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);
//	bus_delay_function();
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
//	bus_delay_function();
}

#if datapins_everywhere

static inline __attribute__((optimize("Ofast"))) void output_D0_D7 (uint8_t b){
if(b&0x01){LCD_ILI9488_D0_SET();}else{LCD_ILI9488_D0_CLR();}
if(b&0x02){LCD_ILI9488_D1_SET();}else{LCD_ILI9488_D1_CLR();}
if(b&0x04){LCD_ILI9488_D2_SET();}else{LCD_ILI9488_D2_CLR();}
if(b&0x08){LCD_ILI9488_D3_SET();}else{LCD_ILI9488_D3_CLR();}
if(b&0x10){LCD_ILI9488_D4_SET();}else{LCD_ILI9488_D4_CLR();}
if(b&0x20){LCD_ILI9488_D5_SET();}else{LCD_ILI9488_D5_CLR();}
if(b&0x40){LCD_ILI9488_D6_SET();}else{LCD_ILI9488_D6_CLR();}
if(b&0x80){LCD_ILI9488_D7_SET();}else{LCD_ILI9488_D7_CLR();}
}

static inline __attribute__((optimize("Ofast"))) void output_D8_D15 (uint8_t b){
if(b&0x01){LCD_ILI9488_D8_SET();}else{LCD_ILI9488_D8_CLR();}
if(b&0x02){LCD_ILI9488_D9_SET();}else{LCD_ILI9488_D9_CLR();}
if(b&0x04){LCD_ILI9488_D10_SET();}else{LCD_ILI9488_D10_CLR();}
if(b&0x08){LCD_ILI9488_D11_SET();}else{LCD_ILI9488_D11_CLR();}
if(b&0x10){LCD_ILI9488_D12_SET();}else{LCD_ILI9488_D12_CLR();}
if(b&0x20){LCD_ILI9488_D13_SET();}else{LCD_ILI9488_D13_CLR();}
if(b&0x40){LCD_ILI9488_D14_SET();}else{LCD_ILI9488_D14_CLR();}
if(b&0x80){LCD_ILI9488_D15_SET();}else{LCD_ILI9488_D15_CLR();}
}
static inline __attribute__((optimize("Ofast"))) void output_D0_D15_port (uint16_t b){
	output_D0_D7((uint8_t)(b & 0x00ff));
	output_D8_D15((uint8_t)(b>>8));
}
#else
static inline __attribute__((optimize("Ofast"))) void output_D0_D15_port (uint16_t b){GPIOB->ODR = b;}
#endif


//--************INIT YAPMADA KULLANILIYOR
void write_control_seq(uint8_t const *cmd_buf){
uint8_t adr, params_q_ty, i;
params_q_ty=cmd_buf[0];
adr=cmd_buf[1];

HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
bus_delay_function();
HAL_GPIO_WritePin(LCD_CD_GPIO_Port,LCD_CD_Pin,GPIO_PIN_RESET);
//bus_delay_function();
output_D0_D15_port((uint16_t)adr);
WritePulse();
HAL_GPIO_WritePin(LCD_CD_GPIO_Port,LCD_CD_Pin,GPIO_PIN_SET);

i=2;
while(params_q_ty){
	output_D0_D15_port((uint16_t)cmd_buf[i]);
	WritePulse();
	i++;
	params_q_ty--;
}
HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

//-**************************************************************

void LCD_ILI9488_init(void){
	HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	write_control_seq(ManufacturerCommandAccessProtect1);
	write_control_seq(ManufacturerCommandAccessProtect2);
	write_control_seq(MemoryAccessInterfaceSetting);
	write_control_seq(DisplayModeFrameMemoryWriteModeSetting);
	write_control_seq(PanelDrivingSetting);
	write_control_seq(DisplayTimingSettingNormalMode);
	write_control_seq(VCOMsetting);
	write_control_seq(InterfaceSetting);
	write_control_seq(GammaSet);
	write_control_seq(Set_address_mode);
	write_control_seq(Set_pixel_format);
	write_control_seq(Exit_idle_mode);
	write_control_seq(PowerSetting);
	write_control_seq(VCOMsetting2);
	write_control_seq(PowerSettingNormalMode);
	write_control_seq(Soft_reset);
	HAL_Delay(200);
	write_control_seq(Set_column_addressR);
	write_control_seq(Set_page_addressR);
	HAL_Delay(100);
	write_control_seq(Set_display_on);
	HAL_Delay(100);
}
//-*******************************************************************************
void Command_2A(uint16_t Xstart,uint16_t Xend)
{
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	//	bus_delay_function();
		HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_RESET);
	//	bus_delay_function();
		output_D0_D15_port(0x002A);//set x range
		WritePulse();
		HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_SET);
		output_D0_D15_port(Xstart>>8);
		WritePulse();
		output_D0_D15_port(Xstart & 0x00ff);
		WritePulse();
		output_D0_D15_port(Xend>>8);
		WritePulse();
		output_D0_D15_port(Xend & 0x00ff);
		WritePulse();
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}
//-*********************************************************************************
void Command_2B(uint16_t Ystart,uint16_t Yend)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
//	bus_delay_function();
	HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_RESET);
//	bus_delay_function();
	output_D0_D15_port(0x002B);//set y range
	WritePulse();
	HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_SET);
	output_D0_D15_port(Ystart>>8);
	WritePulse();
	output_D0_D15_port(Ystart & 0x00ff);
	WritePulse();
	output_D0_D15_port(Yend>>8);
	WritePulse();
	output_D0_D15_port(Yend & 0x00ff);
	WritePulse();
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}
//-********************************************************************************
void Command_2C(void)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
//	bus_delay_function();
	HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_RESET);
//	bus_delay_function();
	output_D0_D15_port(0x002C); //start memory write
	WritePulse();
	HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_SET);
}
//-********************************************************************

//1. prepare coordinates where data will fall in       LCD_ILI9488_raw_datafeed_start
//2. then feed data you have using                     LCD_ILI9488_raw_feed_pixel
//3. then finish operation                             LCD_ILI9488_raw_datafeed_end
__attribute__((optimize("Ofast"))) void LCD_ILI9488_raw_datafeed_start(uint16_t Xstart, uint16_t Xend, uint16_t Ystart, uint16_t Yend){

	if(Xstart>Xend){return;}
	if(Ystart>Yend){return;}
//	if(Xend>=LCD_ILI9488_WidthX){return;}
//	if(Yend>=LCD_ILI9488_WidthY){return;}
//-V-----send X range
	Command_2A(Xstart,Xend);
	bus_delay_function();
//-V----send Y range
	Command_2B(Ystart,Yend);
//	bus_delay_function();
//-V---put buffer contents
	Command_2C();
}
//-************************************************************************
__attribute__((optimize("Ofast"))) void LCD_ILI9488_raw_feed_pixel(uint16_t pixeldata){
			output_D0_D15_port(pixeldata);
			WritePulse();
}
//-*************************************************************************
__attribute__((optimize("Ofast"))) void LCD_ILI9488_raw_datafeed_end(void){
		LCD_ILI9488_CS_SET();
}
//-*************************************************************************************
void LCD_Rect_Fill(uint16_t Xstart, uint16_t Xend, uint16_t Ystart, uint16_t Yend)
{
	uint32_t j = 480*320;
	uint32_t i=0;
	 LCD_ILI9488_raw_datafeed_start(Xstart, Xend-1,Ystart,Yend-1);
	 uint32_t renk=0xf800;
	 renk=renk>>sn;
	for (i = 0; i < j; i++)
	{
				GPIOB->ODR=renk;
				GPIOA->BRR=(uint32_t)LCD_WR_Pin;
				GPIOA->BSRR=(uint32_t)LCD_WR_Pin;
	}
	LCD_ILI9488_raw_datafeed_end();
}

void LCD_Font(uint16_t x, uint16_t y, char *text, const char *p_font, uint32_t backcolor, uint32_t color565)
{
	int16_t cursor_x = x;
	int16_t cursor_y = y;
	uint8_t karekter_start=p_font[2];
	uint8_t karekter_end = p_font[4];
	uint8_t byte_sayisi=p_font[6];

//	memcpy(&font, p_font, sizeof(GFXfont));
	for (uint16_t text_pos = 0; text_pos < strlen(text); text_pos++)
	{
		char c = text[text_pos];
		if (c == '\n')
		{
			cursor_x = x;
//			cursor_y += font.yAdvance * size;
		}
		else if (c >= karekter_start && c <= karekter_end && c != '\r')
		{
		    char Bit_Sayisi=p_font[(c-karekter_start)*4+8];      //karekterin yatay tarama sayısı
			uint16_t karekter_adresi=p_font[(c-karekter_start)*4+9]+p_font[(c-karekter_start)*4+10]*256;
			uint16_t okunacak_byte_sayisi=8;
			uint8_t carpan=1;                                    //her yatay satır için byte sayısı
			if(Bit_Sayisi>=9 && Bit_Sayisi<=16 )
			carpan=2;
			else if(Bit_Sayisi>=17 && Bit_Sayisi<=24 )
			carpan=3;
			else if(Bit_Sayisi>=25 && Bit_Sayisi<=32 )
			carpan=4;

			okunacak_byte_sayisi=carpan*byte_sayisi;

			 LCD_ILI9488_raw_datafeed_start(cursor_x, cursor_x+Bit_Sayisi-1,cursor_y,cursor_y+250);
			 uint32_t Background_adres= 960 * cursor_y + (2 * cursor_x);
			 Background_adres+=sn*307200;
			 uint32_t Def_Background_adres= 960 * cursor_y + (2 * cursor_x);
			 Def_Background_adres +=sn*307200;
			for(uint16_t iy=karekter_adresi;iy<(karekter_adresi+okunacak_byte_sayisi);iy+=carpan)
			{
				uint32_t Data=0;
				if(carpan==1) Data = p_font[iy];
				if(carpan==2) Data=(p_font[iy+1]<<8) + p_font[iy];
				if(carpan==3) Data=(p_font[iy+2]<<16) + (p_font[iy+1]<<8) +p_font[iy];
				if(carpan==4) Data=(p_font[iy+3]<<24) + (p_font[iy+2]<<16) + (p_font[iy+1]<<8) +p_font[iy];
			 for(uint16_t ix=0;ix<Bit_Sayisi;ix++)
			 	{


				 if(Data & 0x000001)
					 GPIOB->ODR=color565;  //karekterın rengı basılıyor
				 else //karekterin background resmi basiliyor
				 {
					 if(backcolor>0xffff)  //renk skalasından buyuk bir deger gonderilice transparan yapılıyor
					 {
						 W25qxx_ReadBytes(buffer,Background_adres,2);
						uint16_t renk=buffer[0]<<8;
						renk |= buffer[1];
						GPIOB->ODR=renk;
					 }
					 else
				       GPIOB->ODR=backcolor;
				 }

			 	GPIOA->BRR=(uint32_t)LCD_WR_Pin;
			 	GPIOA->BSRR=(uint32_t)LCD_WR_Pin;
			 	Data =Data>>1;
				Background_adres+=2;
			 	}
			 Def_Background_adres+=960;
			 Background_adres=Def_Background_adres;

			}
			cursor_x+=Bit_Sayisi;
		}
	}
	LCD_ILI9488_raw_datafeed_end();
}


















/*-------------------------------END OF FILE-------------------------------*/

