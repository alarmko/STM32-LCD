//#ifndef _SSD1331_H_
//#define _SSD1331_H_

/* Includes ------------------------------------------------------------------*/
//#include <math.h>
//#include <inttypes.h>
#include "stm32f0xx_hal.h"
#include "main.h"

#define LCD_ILI9488_WidthX                     320
#define LCD_ILI9488_WidthY                     480



//native nonHAL fast IO
#define LCD_ILI9488_RES_SET()     {LCD_RES_GPIO_Port->BSRR=LCD_RES_Pin;}
//#define LCD_ILI9488_RES_CLR()     {RES_GPIO_Port->BSRR=(uint32_t)RES_Pin<<16U;}
#define LCD_ILI9488_RES_CLR()     {LCD_RES_GPIO_Port->BRR=LCD_RES_Pin;}


#define LCD_ILI9488_WR_SET()     {LCD_WR_GPIO_Port->BSRR=LCD_WR_Pin;}
//#define LCD_ILI9488_WR_CLR()     {WR_GPIO_Port->BSRR=(uint32_t)WR_Pin<<16U;}
#define LCD_ILI9488_WR_CLR()     {LCD_WR_GPIO_Port->BRR=LCD_WR_Pin;}

#define LCD_ILI9488_DATA_CMD_SET()     {LCD_CD_GPIO_Port->BSRR=LCD_CD_Pin;}
//#define LCD_ILI9488_DATA_CMD_CLR()     {CD_GPIO_Port->BSRR=(uint32_t)CD_Pin<<16U;}
#define LCD_ILI9488_DATA_CMD_CLR()     {LCD_CD_GPIO_Port->BRR=LCD_CD_Pin;}

#define LCD_ILI9488_CS_SET()     {LCD_CS_GPIO_Port->BSRR=LCD_CS_Pin;}
//#define LCD_ILI9488_CS_CLR()     {CS_GPIO_Port->BSRR=(uint32_t)CS_Pin<<16U;}
#define LCD_ILI9488_CS_CLR()     {LCD_CS_GPIO_Port->BRR=LCD_CS_Pin;}

/*
#if datapins_everywhere
#define LCD_ILI9488_D0_SET()     {LCD_D0_GPIO_Port->BSRR=LCD_D0_Pin;}
#define LCD_ILI9488_D0_CLR()     {LCD_D0_GPIO_Port->BSRR=(uint32_t)LCD_D0_Pin<<16U;}

#define LCD_ILI9488_D1_SET()     {LCD_D1_GPIO_Port->BSRR=LCD_D1_Pin;}
#define LCD_ILI9488_D1_CLR()     {LCD_D1_GPIO_Port->BSRR=(uint32_t)LCD_D1_Pin<<16U;}

#define LCD_ILI9488_D2_SET()     {LCD_D2_GPIO_Port->BSRR=LCD_D2_Pin;}
#define LCD_ILI9488_D2_CLR()     {LCD_D2_GPIO_Port->BSRR=(uint32_t)LCD_D2_Pin<<16U;}

#define LCD_ILI9488_D3_SET()     {LCD_D3_GPIO_Port->BSRR=LCD_D3_Pin;}
#define LCD_ILI9488_D3_CLR()     {LCD_D3_GPIO_Port->BSRR=(uint32_t)LCD_D3_Pin<<16U;}

#define LCD_ILI9488_D4_SET()     {LCD_D4_GPIO_Port->BSRR=LCD_D4_Pin;}
#define LCD_ILI9488_D4_CLR()     {LCD_D4_GPIO_Port->BSRR=(uint32_t)LCD_D4_Pin<<16U;}

#define LCD_ILI9488_D5_SET()     {LCD_D5_GPIO_Port->BSRR=LCD_D5_Pin;}
#define LCD_ILI9488_D5_CLR()     {LCD_D5_GPIO_Port->BSRR=(uint32_t)LCD_D5_Pin<<16U;}

#define LCD_ILI9488_D6_SET()     {LCD_D6_GPIO_Port->BSRR=LCD_D6_Pin;}
#define LCD_ILI9488_D6_CLR()     {LCD_D6_GPIO_Port->BSRR=(uint32_t)LCD_D6_Pin<<16U;}

#define LCD_ILI9488_D7_SET()     {LCD_D7_GPIO_Port->BSRR=LCD_D7_Pin;}
#define LCD_ILI9488_D7_CLR()     {LCD_D7_GPIO_Port->BSRR=(uint32_t)LCD_D7_Pin<<16U;}

#define LCD_ILI9488_D8_SET()     {LCD_D8_GPIO_Port->BSRR=LCD_D8_Pin;}
#define LCD_ILI9488_D8_CLR()     {LCD_D8_GPIO_Port->BSRR=(uint32_t)LCD_D8_Pin<<16U;}

#define LCD_ILI9488_D9_SET()     {LCD_D9_GPIO_Port->BSRR=LCD_D9_Pin;}
#define LCD_ILI9488_D9_CLR()     {LCD_D9_GPIO_Port->BSRR=(uint32_t)LCD_D9_Pin<<16U;}

#define LCD_ILI9488_D10_SET()     {LCD_D10_GPIO_Port->BSRR=LCD_D10_Pin;}
#define LCD_ILI9488_D10_CLR()     {LCD_D10_GPIO_Port->BSRR=(uint32_t)LCD_D10_Pin<<16U;}

#define LCD_ILI9488_D11_SET()     {LCD_D11_GPIO_Port->BSRR=LCD_D11_Pin;}
#define LCD_ILI9488_D11_CLR()     {LCD_D11_GPIO_Port->BSRR=(uint32_t)LCD_D11_Pin<<16U;}

#define LCD_ILI9488_D12_SET()     {LCD_D12_GPIO_Port->BSRR=LCD_D12_Pin;}
#define LCD_ILI9488_D12_CLR()     {LCD_D12_GPIO_Port->BSRR=(uint32_t)LCD_D12_Pin<<16U;}

#define LCD_ILI9488_D13_SET()     {LCD_D13_GPIO_Port->BSRR=LCD_D13_Pin;}
#define LCD_ILI9488_D13_CLR()     {LCD_D13_GPIO_Port->BSRR=(uint32_t)LCD_D13_Pin<<16U;}

#define LCD_ILI9488_D14_SET()     {LCD_D14_GPIO_Port->BSRR=LCD_D14_Pin;}
#define LCD_ILI9488_D14_CLR()     {LCD_D14_GPIO_Port->BSRR=(uint32_t)LCD_D14_Pin<<16U;}

#define LCD_ILI9488_D15_SET()     {LCD_D15_GPIO_Port->BSRR=LCD_D15_Pin;}
#define LCD_ILI9488_D15_CLR()     {LCD_D15_GPIO_Port->BSRR=(uint32_t)LCD_D15_Pin<<16U;}
#else

#endif


#define RGB(R,G,B)  (((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3))
enum Color{
    BLACK     = RGB(  0,  0,  0), // black
    GREY      = RGB(127,127,127), // grey
    WHITE     = RGB(255,255,255), // white
    RED       = RGB(255,  0,  0), // red
    PINK      = RGB(255,0  ,127), // pink
    YELLOW    = RGB(255,255,  0), // yellow
    GOLDEN    = RGB(255,215,  0), // golden
    BROWN     = RGB(128, 42, 42), // brown
    BLUE      = RGB(  0,  0,255), // blue
    CYAN      = RGB(  0,255,255), // cyan
    GREEN     = RGB(  0,255,  0), // green
    PURPLE    = RGB(160, 32,240), // purple
};

*/





/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/                    

/* Exported functions ------------------------------------------------------- */
//extern void LCD_ILI9488_clear_screen(uint16_t hwColor);
extern void LCD_ILI9488_init(void);



//fast memory filler kit.
//IF CALL SEQUENCE GETS WRONG, OR YOU FORGET SOMETHING, ALL WILL BE FUCKED UP. BEWARE!!!
void LCD_ILI9488_raw_datafeed_start(uint16_t Xstart, uint16_t Xend, uint16_t Ystart, uint16_t Yend);
void LCD_ILI9488_raw_feed_pixel(uint16_t pixeldata);
void LCD_ILI9488_raw_datafeed_end(void);
void Command_2A(uint16_t Xstart,uint16_t Xend);
void Command_2B(uint16_t Ystart,uint16_t Yend);
void Command_2C(void);
void LCD_Font(uint16_t x, uint16_t y, char *text, const char *p_font, uint32_t backcolor, uint32_t color565);

typedef struct { // Data stored PER GLYPH
	uint16_t bitmapOffset;     // Pointer into GFXfont->bitmap
	uint8_t  width, height;    // Bitmap dimensions in pixels
	uint8_t  xAdvance;         // Distance to advance cursor (x axis)
	int8_t   xOffset, yOffset; // Dist from cursor position to UL corner
} GFXglyph;

typedef struct { // Data stored for FONT AS A WHOLE:
	uint8_t  *bitmap;      // Glyph bitmaps, concatenated
	GFXglyph *glyph;       // Glyph array
	uint8_t   first, last; // ASCII extents
	uint8_t   yAdvance;    // Newline distance (y axis)
} GFXfont;


/*-------------------------------END OF FILE-------------------------------*/

