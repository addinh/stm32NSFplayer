/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

#include "nes_channels.h"
#include "emulator6502.h"
#include "nsf_array.h"
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
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
#define DAC_BUFFER_LEN 256
uint16_t DAC_BUFFER[DAC_BUFFER_LEN] = {0};

//debug
//uint8_t dummy = 0;
uint8_t op_debug = 0;
int16_t button_counter = 0;
uint8_t SW1_pressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// TODO move to separate files
inline void overflowCheck(uint16_t *buffer, uint16_t buffer_len) {
	//overflow check
	//TODO scale with AMPLITUDE_SCALE
	for (uint16_t i = 0; i < buffer_len; ++i) {
		buffer[i] = (uint16_t)(buffer[i] * 1.5f);
		if (buffer[i] > 4095) buffer[i] = 4095;
	}
};

// DMA INTERRUPTS
void update_DAC_BUFFER(uint16_t *buffer, uint16_t len) {
	//debug
	//if (button_counter != 25) return;
	memset(buffer, 0, len * 2);
	readPulse(buffer, len, &pulse1);
	readPulse(buffer, len, &pulse2);
	readTriangle(buffer, len, &triangle);
	readNoise(buffer, len, &noise);
	overflowCheck(buffer, len);
	//dummy = 1;	
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	update_DAC_BUFFER(&DAC_BUFFER[0], DAC_BUFFER_LEN/2);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	update_DAC_BUFFER(&DAC_BUFFER[DAC_BUFFER_LEN/2], DAC_BUFFER_LEN/2);
}

void initRAM(void) {
	memset(pRAM, 0, 0x0800);
	memset(pSRAM, 0, 0x2000);
	memset(pAPU, 0, 0x0020);
	memset(pExRAM, 0, 0x1000);
	memset(pROM_Full, 0, 0x8000);
	pStack = pRAM + 0x100;
	
	int idx = 0xbdc4;
	for (uint32_t i=0; i<16956; ++i) {
		pROM_Full[idx - 0x8000 + i] = NSF_ARRAY[i];
	}
	//DEBUG PLS REMOVE
	pROM_Full[0] = 0x69;
	
	pExRAM[0x00] = 0x20;						//JSR
	uint16_t init_addr = 0xbe34;
	memcpy(&pExRAM[0x01],&init_addr,2);	//Init Address
	pExRAM[0x03] = 0xF2;						//JAM
	pExRAM[0x04] = 0x20;						//JSR
	uint16_t play_addr = 0xf2d0;
	memcpy(&pExRAM[0x05],&play_addr,2);	//Play Address
	pExRAM[0x07] = 0x4C;						//JMP
	pExRAM[0x08] = 0x03;						//$5003  (JAM right before the JSR to play address)
	pExRAM[0x09] = 0x50;

	regA = regX = regY = 0;
	regP = 0x04;			//I_FLAG
	regSP = 0xFF;

	/*	Reset Read/Write Procs			*/

	ReadMemory[0] = ReadMemory[1] = ReadMemory_RAM;
	ReadMemory[2] = ReadMemory[3] = ReadMemory_Default;
	ReadMemory[4] =					ReadMemory_pAPU;
	ReadMemory[5] =					ReadMemory_ExRAM;
	ReadMemory[6] = ReadMemory[7] = ReadMemory_SRAM;

	WriteMemory[0] = WriteMemory[1] =	WriteMemory_RAM;
	WriteMemory[2] = WriteMemory[3] =	WriteMemory_Default;
	WriteMemory[4] =					WriteMemory_pAPU;
	WriteMemory[5] =					WriteMemory_ExRAM;
	WriteMemory[6] = WriteMemory[7] =	WriteMemory_SRAM;

	for(int i = 8; i < 16; i++) {
		ReadMemory[i] = ReadMemory_ROM;
		WriteMemory[i] = WriteMemory_Default;
	}	

}

void initCPU() {
	regPC = 0x5000;
	regA = 3 - 1;
	regX = bPALMode;
	regY = bCleanAXY ? 0 : 0xCD;
	regSP = 0xFF;
	if(bCleanAXY)
		regP = 0x04;
	bCPUJammed = 0;

	nCPUCycle = nAPUCycle = 0;

	//TODO?
	for(int i = 0x4000; i < 0x400F; i++)
		WriteMemory_pAPU(i,0);
	WriteMemory_pAPU(0x4010,0);
	WriteMemory_pAPU(0x4012,0);
	WriteMemory_pAPU(0x4013,0);
	WriteMemory_pAPU(0x4014,0);
	WriteMemory_pAPU(0x4015,0);
	WriteMemory_pAPU(0x4015,0x0F);
	WriteMemory_pAPU(0x4017,0);

	for(int i = 0; i < 10; i++) {
		//WriteMemory_ExRAM(0x5FF6 + i, nBankswitchInitValues[i]);

		//if using Famitracker's ROM method
		//from current understanding, for example load_addr is bdc4
		//then pROM[0] to pROM[5] points to pROM_Full[0x0...]
		//pROM[6] points to pROM_Full[0x1...] and so on
		//pROM[i] = pROM_Full + (nBankswitchInitValues[i] << 12);

		//if pROM_Full is precisely 0x8000-0xFFFF
		//pROM[2] points to pROM_Full[0x0...], pROM[3] points to pROM_Full[0x1...] and so on
		//ignore 0 and 1 for now, point to SRAM instead
		if (i >= 2) pROM[i] = pROM_Full + ((i-2) << 12);
		else pROM[i] = pSRAM + (i << 12);
	}
}
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
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	initRAM();
	initCPU();
	initTriangleLookup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)DAC_BUFFER, DAC_BUFFER_LEN, DAC_ALIGN_12B_R);
	
	pulse1 = (PulseChannel){
		.volume = 0,
	};
	
	pulse2 = (PulseChannel) {
		.volume = 0,
	};
	
	triangle = (TriangleChannel){
		.timer_lo = 0,
	};
	
	noise = (NoiseChannel){
		.shift_register = 1,
		.volume = 0,
	};
	
	//do init instructions
	while (regPC != 0x5003) { //until PC reaches after INIT
		//execute next instruction
		emulate6502(nCPUCycle + 1);
		if (regPC < 0x5000 || (regPC > 0x5009 && regPC < 0x8000)) {
			button_counter = -1;
			break;
		}
	}
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			
		//Wave generation thread
		static uint32_t last_wav_ticks = 0;
		static uint32_t prev_button_counter = 0;
		//TODO update time = frame rate
		//TODO trying at 1Hz
		if (HAL_GetTick() - last_wav_ticks >= 16) {
			last_wav_ticks = HAL_GetTick();
			
			static uint8_t update = 1;
			
			if(button_counter && button_counter != -1) {
				while (!(regPC > 0x5FFF)) { //until PC reaches PLAY instructions
					//execute next instruction
					emulate6502(nCPUCycle + 1);
					if (regPC < 0x5000 || (regPC > 0x5009 && regPC < 0x8000) || (bCPUJammed == 1)) {
						button_counter = -1;
						break;
					}
				}
				while (regPC != 0x5007) { //until PC finishes PLAY instructions
					//execute next instruction
					emulate6502(nCPUCycle + 1);
					if (regPC < 0x5000 || (regPC > 0x5009 && regPC < 0x8000) || (bCPUJammed == 1)) {
						button_counter = -1;
						break;
					}
				}
				update = 1;
			}
			
			if (update) {
				//link pAPU registers to wave structs
				pulse1.DDLCVVVV = pAPU[0];
				pulse1.EPPPNSSS = pAPU[1];
				pulse1.TTTTTTTT = pAPU[2];
				pulse1.LLLLLTTT = pAPU[3];
				pulse1.wave.enable = pAPU[21] & 0x01;
			
				pulse2.DDLCVVVV = pAPU[4];
				pulse2.EPPPNSSS = pAPU[5];
				pulse2.TTTTTTTT = pAPU[6];
				pulse2.LLLLLTTT = pAPU[7];
				pulse1.wave.enable = pAPU[21] & 0x02;
			
				triangle.CRRRRRRR = pAPU[8];
				//triangle.UUUUUUUU = pAPU[9];
				triangle.TTTTTTTT = pAPU[10];
				triangle.LLLLLTTT = pAPU[11];
				pulse1.wave.enable = pAPU[21] & 0x04;
			
				noise.UULCVVVV = pAPU[12];
				//noise.UUUUUUUU = pAPU[13];
				noise.MUUUPPPP = pAPU[14];
				noise.LLLLLUUU = pAPU[15];
				pulse1.wave.enable = pAPU[21] & 0x08;
			
				//update corresponding params
				updatePulse(&pulse1);
				updatePulse(&pulse2);
				updateTriangle(&triangle);
				//updateNoise(&noise);		
				
				update = 0;
			}
			
			/*
			//if (button_counter != 1) button_counter += 1;
			if (prev_button_counter != button_counter) {
				//call at beginning because a section below may update button_counter for autoplay
				prev_button_counter = button_counter;
				
				switch (button_counter) {
					//pulse demo
					case 1: setPulseTimer(&pulse1, 858); pulse1.duty = 0; pulse1.volume = 15; break;
					case 2: setPulseTimer(&pulse1, 764); pulse1.duty = 1; pulse1.volume = 14; break; 
					case 3: setPulseTimer(&pulse1, 681); pulse1.duty = 2; pulse1.volume = 12; break; 
					case 4: setPulseTimer(&pulse1, 642); pulse1.duty = 3; pulse1.volume = 11; break; 
					case 5: setPulseTimer(&pulse1, 573); pulse1.duty = 2; pulse1.volume = 9; break; 
					case 6: setPulseTimer(&pulse1, 510); pulse1.duty = 1; pulse1.volume = 8; break; 
					case 7: setPulseTimer(&pulse1, 454); pulse1.duty = 0; pulse1.volume = 6; break; 
					case 8: setPulseTimer(&pulse1, 428); pulse1.duty = 2; pulse1.volume = 1; break;
					case 9: pulse1.volume = 0; break;

					//triangle demo
					case 10: setTriangleTimer(&triangle, 429); break;
					case 11: setTriangleTimer(&triangle, 382); break; 
					case 12: setTriangleTimer(&triangle, 341); break; 
					case 13: setTriangleTimer(&triangle, 321); break; 
					case 14: setTriangleTimer(&triangle, 287); break; 
					case 15: setTriangleTimer(&triangle, 255); break; 
					case 16: setTriangleTimer(&triangle, 227); break;
					case 17: setTriangleTimer(&triangle, 0); break;
				
					//noise demo
					case 18: noise.volume = 15; noise.period_index = 15; break; 
					case 19: noise.volume = 14; noise.period_index = 13; break; 
					case 20: noise.volume = 13; noise.period_index = 11; break; 
					case 21: noise.volume = 12; noise.period_index = 9; break; 
					case 22: noise.volume = 11; noise.period_index = 7; break; 
					case 23: noise.volume = 10; noise.period_index = 5; break;
					case 24: noise.volume = 0; break;
					
					//mixing demo
					case 25:
						setPulseTimer(&pulse1, 573); pulse1.duty = 0; pulse1.volume = 15;
						setPulseTimer(&pulse2, 214); pulse2.duty = 2; pulse2.volume = 10;
						setTriangleTimer(&triangle, 429);
						noise.volume = 7; noise.period_index = 11;
						break;
				
					default: break;
				}
				//TODO these should be called when respective channels are updated
				updatePulse(&pulse1);
				updatePulse(&pulse2);
				updateTriangle(&triangle);
				updateNoise(&noise);
			}*/
			
		}
		
		//LCD thread
		static uint32_t last_lcd_ticks = 0;
		if (HAL_GetTick() - last_lcd_ticks >= 50) {
			last_lcd_ticks = HAL_GetTick();
			/*LCD_Clear(5 * WIDTH_EN_CHAR, 1 * HEIGHT_EN_CHAR, 5 * WIDTH_EN_CHAR, 3 * HEIGHT_EN_CHAR, BACKGROUND);

			// print debug
			print_int_with_label(1 * WIDTH_EN_CHAR, 1 * HEIGHT_EN_CHAR, "Ticks: ", 7, last_lcd_ticks, "%d");
			//print_int_with_label(1 * WIDTH_EN_CHAR, 5 * HEIGHT_EN_CHAR, "p1_am: ", 7, pulse1.wave.amplitude, "%d");
			//print_int_with_label(1 * WIDTH_EN_CHAR, 6 * HEIGHT_EN_CHAR, "p1_pe: ", 7, pulse1.wave.period, "%d");
			//print_int_with_label(1 * WIDTH_EN_CHAR, 8 * HEIGHT_EN_CHAR, "p1_co: ", 7, pulse1.wave.counter, "%d");
			print_int_with_label(1 * WIDTH_EN_CHAR, 2 * HEIGHT_EN_CHAR, "OUT: ", 5, HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1), "%d");
			print_int_with_label(1 * WIDTH_EN_CHAR, 3 * HEIGHT_EN_CHAR, "State: ", 7, button_counter, "%d");
			//print_int_with_label(1 * WIDTH_EN_CHAR, 5 * HEIGHT_EN_CHAR, "debug: ", 7, dummy, "%d");*/
		}
		
		//LED thread
		static uint32_t last_led_ticks = 0;
		if (HAL_GetTick() - last_led_ticks >= (button_counter == -1 ? 50 : 100 * (4 - (button_counter % 4)))) {
			last_led_ticks = HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		}
		
		//button thread
		static uint32_t last_button_ticks = 0;
		if (HAL_GetTick() - last_button_ticks >= 50) {
			last_button_ticks = HAL_GetTick();
			if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) && !SW1_pressed) {
				SW1_pressed = 1;
				button_counter += 1;
			}
			else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) && SW1_pressed) {
				SW1_pressed = 0;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 93;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
