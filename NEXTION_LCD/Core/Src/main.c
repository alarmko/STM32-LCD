/* USER CODE BEGIN Header */
/** FONT DOSYASINI SHORT TAN CHAR A CEVIR YOKSA HAFIZADA COK YER KAPLIYOR VE CALISMAZ
 * 05-11-2019 da hazırlandı
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "fatfs_sd.h"
//#include "string.h"
//#include "math.h"
#include "ILI9488.h"
#include "w25qxx.h"
#include "BULENFONT.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS fs;  //file system
FIL fil;   // to store the result
FRESULT fresult ; //to store the result
uint8_t buffer[2880]; // to store data
uint8_t txbuffer[5];
UINT br,bw;  // file read/write count
FILINFO fi;
DIR dir;

/*capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

uint16_t renk;
uint32_t top_adet=0;
uint8_t sn=0;

uint8_t tusa_basildi=0;
uint8_t tus_algilama_sayisi=0;

uint16_t TS_KOR_X=0;
uint16_t TS_KOR_Y=0;

uint16_t X_MAX=0;
uint16_t X_MIN=0;
uint16_t Y_MAX=0;
uint16_t Y_MIN=0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LCD_RD_GPIO_Port, LCD_RD_Pin, GPIO_PIN_SET);  //OKUMA MODU KAPATILIYOR
	LCD_ILI9488_init();
	W25qxx_Init();

  /*to find the size of data in the buffer */

  /*
  int buffersize (char *buf)
  {
  	int i=0;
  	while(*buf++ != '\0') i++;
  	return i;

  }

*/
  void bufferclear (void)  //clear buffer
  {
  	for (int i=0; i<sizeof(buffer); i++)
  	{
  		//buffer[i]= '\0';
  		buffer[i]= 0XFF;  // BOS YERLER FF LE DOLDURULUYOR
  	}
  }
  //***************************************
  void SD_TO_W25Q()  //BUTUN RESIMLERI TEK SEFERDE KAYDEDER
  {

	  bufferclear();
      char dosyaadi[]="00.bin";   //DOSYA ADLARI "00.bin" den BASLAYIP "99.bin" 'E KADAR
      uint8_t dosyano=0;
      uint16_t sayfano=0;
      uint16_t sectorno=0;
      uint16_t def_sectorno=0;
      W25qxx_EraseSector(0);

  		LCD_Font(10, 10, "SD  KART  BULUNDU  DOSYALAR  KAYDEDILIYOR\n", Segoe_Script21x21,0xffff, 0xf800);
//  		W25qxx_EraseChip();     //KAYIT ISLEMININ YAPILABILMESI ICIN ONCELIKLE SILINMESI GEREKIYOR

  		TEKRAR:
  		fresult =f_open(&fil, dosyaadi, FA_READ);	    /* open file to read FA_READ */

  		if(fresult == FR_OK)
  		{
  		/* Read string from the file */

  		top_adet=0;
  		do
  		{
  		f_read(&fil, buffer, 256, &br);
  		W25qxx_WritePage(buffer,sayfano,0,256);  //kaydederken 1 2 3 4 sayfa no yazılacak(8 BİT)
 // 		W25qxx_ReadBytes(buffer,sayfano<<8,256); //okurken gercek adres yazılacak(32 BİT)
  		sayfano ++;
        sectorno=sayfano>>4;
        if(sectorno != def_sectorno)
        {
        	W25qxx_EraseSector(sectorno);
        	def_sectorno=sectorno;
        }
  		top_adet+=256;
  		}while(top_adet<fil.fsize);

  		f_close(&fil);	 /*close file */
  		HAL_Delay(1000);
  		}

  		dosyano++;
  		dosyaadi[1]=dosyano%10+48;
  		dosyaadi[0]=dosyano/10 +48;
  		if (dosyano <99) goto TEKRAR;
  	    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  		LCD_Font(10, 30, "BITTI  sd karti cikartiniz.\n", Segoe_Script21x21,0xffff, 0x001F);

  	}



//***************************************
void WQ25_TO_LCD(uint8_t Resim_Id,uint16_t Xstart, uint16_t Xend, uint16_t Ystart, uint16_t Yend)
{

//	uint32_t okuma_bas=(Ystart*480*2+Xstart)+(Resim_Id*1200*256); //son kısım sayfa secmede kullanılıyor
	uint32_t okuma_bas=(Ystart*480 +Xstart)*2+(Resim_Id*1200*256); //son kısım sayfa secmede kullanılıyor
	uint32_t okuma_bit=Yend*480+Xend;

	uint16_t adet=(Xend-Xstart)*6;

	    LCD_ILI9488_raw_datafeed_start(Xstart, Xend-1,Ystart,Yend-1);

		for(uint16_t yi=Ystart;yi<=Yend;yi+=3)
		{
		W25qxx_ReadBytes(buffer,okuma_bas,adet); //okurken gercek adres yazılacak
		for(uint16_t xi=0;xi<adet;xi+=2)
		{
		renk=buffer[xi]<<8;
		renk |= buffer[xi+1];
		GPIOB->ODR=renk;
		GPIOA->BRR=(uint32_t)LCD_WR_Pin;
		GPIOA->BSRR=(uint32_t)LCD_WR_Pin;
		}
		okuma_bas+=2880;  //her satır 480*2=960 byte den olusuyor
		}

		LCD_ILI9488_raw_datafeed_end();
}
//-*******************************************************
//-*******************************************************
void RENK_TO_LCD(uint16_t renk, uint16_t Xstart, uint16_t Xend, uint16_t Ystart, uint16_t Yend)
{


	    uint16_t adet=(Xend-Xstart)*2;

	    LCD_ILI9488_raw_datafeed_start(Xstart, Xend-1,Ystart,Yend-1);

		for(uint16_t yi=Ystart;yi<=Yend;yi++)
		{
		for(uint16_t xi=0;xi<adet;xi+=2)
		{
		GPIOB->ODR=renk;
		GPIOA->BRR=(uint32_t)LCD_WR_Pin;
		GPIOA->BSRR=(uint32_t)LCD_WR_Pin;
		}
		}

		LCD_ILI9488_raw_datafeed_end();

}
//-*******************************************************
void SD_KART_KONTROL()
{
	  if(!HAL_GPIO_ReadPin(SD_TAKILDI_GPIO_Port, SD_TAKILDI_Pin))
	  {

	  fresult =f_mount(&fs, "",0);  //SD KART TAKILDIMI
	  	if(fresult == FR_OK)
	  	{
	  	SD_TO_W25Q();
	  	}
	  	while(!HAL_GPIO_ReadPin(SD_TAKILDI_GPIO_Port, SD_TAKILDI_Pin));
	  	NVIC_SystemReset();
	  }
}
//************************************************************************
void TS_KALIBRE()
{
	    TS_KOR_X=0;
	    RENK_TO_LCD(0, 0, 480, 0, 320);
	    while (!tusa_basildi)
	    {
	     RENK_TO_LCD(0xf800, 0, 20, 0, 20);
	    }
	    tusa_basildi=0;
	    TS_KOR_XY_OKU();
	    X_MAX=TS_KOR_X;
	    Y_MAX=TS_KOR_Y;
	    RENK_TO_LCD(0, 0, 480, 0, 320);
//******
	    TS_KOR_X=0;
	    while (!tusa_basildi)
	    {
	     RENK_TO_LCD(0xf800, 460, 480, 0, 20);
	    }
	    tusa_basildi=0;
	     TS_KOR_XY_OKU();
	    RENK_TO_LCD(0, 0, 480, 0, 320);
//*******
	    TS_KOR_X=0;
	    while (!tusa_basildi)
	    {
	     RENK_TO_LCD(0xf800, 460, 480, 300, 320);
	    }
	    tusa_basildi=0;
	    TS_KOR_XY_OKU();
	    X_MIN=TS_KOR_X;
	  	Y_MIN=TS_KOR_Y;
	    RENK_TO_LCD(0, 0, 480, 0, 320);
//*******
	    TS_KOR_X=0;
	    while (!tusa_basildi)
	    {
	     RENK_TO_LCD(0xf800, 0, 20, 300, 320);
	    }
	    tusa_basildi=0;
	    TS_KOR_XY_OKU();
	    HAL_Delay(2000);
	    RENK_TO_LCD(0, 0, 480, 0, 320);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxRxCpltCallback should be implemented in the user file
   */

}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
HAL_GPIO_WritePin(W25Q_CS_GPIO_Port, W25Q_CS_Pin, GPIO_PIN_SET); //W25Q SPI ICIN KAPATILIYOR
HAL_GPIO_WritePin(LCD_BACKLIGHT_GPIO_Port, LCD_BACKLIGHT_Pin,GPIO_PIN_SET);

HAL_Delay(500);
SD_KART_KONTROL();

//	W25qxx_WriteStatusRegister(2,0);
HAL_Delay(2000);
//TS_KALIBRE();

txbuffer[0]=0x0b;
txbuffer[1]=0;
txbuffer[2]=0;
txbuffer[3]=0;
txbuffer[4]=0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  SD_KART_KONTROL();

    //RENK_TO_LCD(0xf800>>sn,0,480,0,320);
//   RENK_TO_LCD(0XF800, 0, 480, 0, 320);
/*
		HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	//	bus_delay_function();
		HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_RESET);
	//	bus_delay_function();
		GPIOB->ODR = (0x0028);//set y range
		HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);
	//	bus_delay_function();
		HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_SET);

*/
 //	WQ25_TO_LCD(sn, 0, 480, 0, 320);

/*
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
//	bus_delay_function();
	HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_RESET);
//	bus_delay_function();
	GPIOB->ODR = (0x0029);//set y range
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);
//	bus_delay_function();
	HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CD_GPIO_Port, LCD_CD_Pin, GPIO_PIN_SET);
*/
	/*
    const bb[]={sn+48 ,"\n"};
    LCD_Font(460, 50*sn, bb , Segoe_Script21x21, 0x1ffff, 0xf800);
    LCD_Font(100, 50*sn, "BULENT       ASLANTURK\n", Segoe_Script21x21, 0x107e0, 0xf800);
 	HAL_Delay(1000);
*/
 //	LCD_Rect_Fill(0, 480, 0, 320);
// 	HAL_Delay(1000);
// 	W25qxx_ReadBytes_BULENT(&sn, top_adet, 2);
 	sn++;
 	if (sn>5) sn=0;

 	if(tusa_basildi)
 	 	{
 		TS_KOR_XY_OKU();
 		tusa_basildi=0;
 		tus_algilama_sayisi++;
 	 	}


 	if(tus_algilama_sayisi >1)
 	{
 	RENK_TO_LCD(0XF800, ((3860-TS_KOR_X)*0.129729), ((3860-TS_KOR_X)*0.129729)+10, ((3800-TS_KOR_Y)*0.086486), ((3800-TS_KOR_Y)*0.086486)+10);
 	tus_algilama_sayisi=0;
 	}



  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TS_DIN_Pin|LCD_BACKLIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TS_CS_Pin|TS_DCLCK_Pin|W25Q_CS_Pin|LCD_RD_Pin 
                          |LCD_WR_Pin|LCD_CD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_CS_Pin|LCD_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_PENIRQ_Pin */
  GPIO_InitStruct.Pin = TS_PENIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TS_PENIRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_TAKILDI_Pin */
  GPIO_InitStruct.Pin = SD_TAKILDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_TAKILDI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_DOUT_Pin */
  GPIO_InitStruct.Pin = TS_DOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TS_DOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TS_DIN_Pin */
  GPIO_InitStruct.Pin = TS_DIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TS_DIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TS_CS_Pin TS_DCLCK_Pin */
  GPIO_InitStruct.Pin = TS_CS_Pin|TS_DCLCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : W25Q_CS_Pin LCD_RD_Pin LCD_WR_Pin LCD_CD_Pin */
  GPIO_InitStruct.Pin = W25Q_CS_Pin|LCD_RD_Pin|LCD_WR_Pin|LCD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BACKLIGHT_Pin */
  GPIO_InitStruct.Pin = LCD_BACKLIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_BACKLIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_RES_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  tusa_basildi=1;

  /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
