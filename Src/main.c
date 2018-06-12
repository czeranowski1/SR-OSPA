/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "dac.h"
#include "dfsdm.h"
#include "dma.h"
#include "i2c.h"
#include "lcd.h"
#include "quadspi.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_audio.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "stm32l476g_discovery_qspi.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define WAV_HEADER_SIZE 44

struct WavHeader{
	char chunkID[4];
	uint32_t chunksize;
	char format[4];
	char subchunk1ID[4];
	uint32_t subchunk1Size;
	uint16_t audioFormat;
	uint16_t numofchannels;
	uint32_t sampleRate;
	uint32_t byteRate;
	uint16_t blockAlign;
	uint16_t bitsPerSample;
	char subchunk2ID[4];
	uint32_t subchink2Size;

};



uint16_t buff_size = 4096; //in bytes
uint32_t start=0;
uint32_t read=0;
uint32_t end=0;;
uint32_t btr=0;
uint16_t buff[2048];

uint8_t complete_flag=0;
uint8_t half_complete_flag=0;

uint8_t isPlaying=0;
uint8_t isPaused=0;
uint8_t volume = 50;

uint8_t receiv = 0;

void AudioErr(){}
void AudioHalf(){

	if(read+buff_size/2<= end)
	    		btr = buff_size/2;
	    	else
	    		btr = end-read;
	half_complete_flag=1;
	BSP_QSPI_Read((uint8_t*)&buff[0], read, btr);
	    		read+=btr;

	        	if(btr!=buff_size/2){ //end of file
	        		BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
	        		isPlaying=0;
	        		read = start+44;

	        	}


}


void AudioCplt(){

if(read+buff_size/2<= end)
    		btr = buff_size/2;
    	else
    		btr = end-read;

complete_flag=1;
BSP_QSPI_Read((uint8_t*)&buff[buff_size/4], read, btr);
read+=btr;

if(btr!=buff_size/2){ //end of file
	BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
	isPlaying=0;
	read = start+44;

}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if(huart == &huart2){
		receiv=1;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_LCD_Init();
  MX_QUADSPI_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  MX_DAC1_Init();
  MX_DFSDM1_Init();
  MX_SAI1_Init();
  /* USER CODE BEGIN 2 */
    char vol_message[6]; //audio volume info


    BSP_QSPI_Init();
    BSP_LCD_GLASS_Init();
    BSP_LCD_GLASS_Clear();

    struct WavHeader header;
    BSP_QSPI_Read((uint8_t*)&header, 0, WAV_HEADER_SIZE);

    if(header.numofchannels==1){
 	   header.numofchannels = BSP_AUDIO_OUT_MONOMODE;
    }
    else{
 	   header.numofchannels = BSP_AUDIO_OUT_STEREOMODE;
    }

    start = 0+WAV_HEADER_SIZE;
    read = start;
    end = 0+header.chunksize+8;

    BSP_QSPI_Read((uint8_t*)&buff[0], start, buff_size);
    read+=buff_size;

    BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 50, header.sampleRate*2);

    BSP_AUDIO_OUT_ChangeAudioConfig(BSP_AUDIO_OUT_CIRCULARMODE | header.numofchannels);

    BSP_AUDIO_OUT_RegisterCallbacks(AudioErr, AudioHalf, AudioCplt);

    BSP_LCD_GLASS_DisplayString((uint8_t*)"PLAY");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */


    if(HAL_GPIO_ReadPin(JOY_CENTER_GPIO_Port, JOY_CENTER_Pin)){
        	while(HAL_GPIO_ReadPin(JOY_CENTER_GPIO_Port, JOY_CENTER_Pin)){}
        	if(!isPlaying){
        		if(BSP_AUDIO_OUT_Play(&buff[0], buff_size) == AUDIO_OK){
        			isPlaying=1;
        			BSP_LCD_GLASS_DisplayString((uint8_t*)"PLAYIN");
        		}

        		else{
        			BSP_LCD_GLASS_ScrollSentence((uint8_t*)"AUDIO ERROR", 1, 200);
        		}
        	}
        	else{
        		if(isPaused==0){
        		BSP_AUDIO_OUT_Pause();
        		BSP_LCD_GLASS_Clear();
        		BSP_LCD_GLASS_DisplayString((uint8_t*)"PAUSE");
        		isPaused=1;
        		}
        		else{
        			BSP_AUDIO_OUT_Resume();
        			isPaused=0;
        			BSP_LCD_GLASS_DisplayString((uint8_t*)"PLAYIN");
        		}
        	}

        }


        if(HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, JOY_LEFT_Pin)){
        	while(HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, JOY_LEFT_Pin)){}
        	HAL_GPIO_TogglePin(LD_R_GPIO_Port, LD_R_Pin);
        	 if(volume>0){
        		 volume--;
        	 }
        	 BSP_AUDIO_OUT_SetVolume(volume);
        	 sprintf(vol_message, "VOL:%d", volume);
        	 BSP_LCD_GLASS_DisplayString((uint8_t*)vol_message);
        }

        else if(HAL_GPIO_ReadPin(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin)){
        	while(HAL_GPIO_ReadPin(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin)){}
        	HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
        	if(volume<100){
        		volume++;
        	}
        	BSP_AUDIO_OUT_SetVolume(volume);
        	sprintf(vol_message, "VOL:%d", volume);
        	BSP_LCD_GLASS_DisplayString((uint8_t*)vol_message);
        }
        if(HAL_GPIO_ReadPin(JOY_DOWN_GPIO_Port, JOY_DOWN_Pin)){
        	while(HAL_GPIO_ReadPin(JOY_RIGHT_GPIO_Port, JOY_RIGHT_Pin)){}
    	if(isPlaying){
        	BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
    		isPlaying=0;
    		read = start+44;
    		BSP_LCD_GLASS_Clear();
    		BSP_LCD_GLASS_DisplayString((uint8_t*)"STOP");
    	}
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV17;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI2;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI2.PLLSAI2Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI2.PLLSAI2M = 1;
  PeriphClkInit.PLLSAI2.PLLSAI2N = 34;
  PeriphClkInit.PLLSAI2.PLLSAI2P = RCC_PLLP_DIV17;
  PeriphClkInit.PLLSAI2.PLLSAI2R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
