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
#include "stdlib.h"

#include "nes_channels.h"
#include "emulator6502.h"
#include "nsf_array.h"
#include "button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED1 GPIOC, GPIO_PIN_1
#define LED2 GPIOC, GPIO_PIN_2
#define LED3 GPIOC, GPIO_PIN_3

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
uint8_t song_loaded = 0;
uint8_t song_select = 0;
uint8_t num_songs = 18;
uint8_t led_bug = 0;

//NES channels
PulseChannel pulse1 = {
	.volume = 0,
};
	
PulseChannel pulse2 = {
	.volume = 0,
};
	
TriangleChannel triangle = {
	.timer_lo = 0,
};
	
NoiseChannel noise = {
	.shift_register = 1,
	.volume = 0,
};
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
#define AUDIO_VOLUME 0.25f

void mixAudio(uint16_t *buffer, uint8_t *mix_buffer, uint16_t len) {
	//perform mixing on a buffer 4 times the length of (half) output to DAC buffer
	//overall scaling done here
	//note that len should be the length of the output buffer aka 1/4 length of buffer
	for (uint16_t i = 0; i < len; ++i) {
		#define p1 mix_buffer[i]
		#define p2 mix_buffer[len + i]
		#define tr mix_buffer[2*len + i]
		#define ns mix_buffer[3*len + i]
		
		//pulse mixing
		float pulseMix = (p1 + p2) == 0 ? 0 : (95.88f / (8128.0f / (p1 + p2) + 100));
		//tri and noise mixing
		float tnMix = (tr + ns) == 0 ? 0 : (159.79f / (1.0f / (tr / 8227.0f + ns / 12241.0f) + 100));
		
		//put in buffer with scaling
		buffer[i] = (uint16_t)((pulseMix + tnMix) * 4095 * AUDIO_VOLUME);
		
		//overflow check
		if (buffer[i] > 4095) buffer[i] = 4095;
	}
}

//going safe method here
uint8_t mix_buffer[DAC_BUFFER_LEN * 4 / 2] = {0};

// DMA INTERRUPTS
void update_DAC_BUFFER(uint16_t *buffer, uint16_t len) {
	//create a temp array for mixing
	//uint8_t* mix_buffer = (uint8_t*)malloc(len * 4);
	memset(mix_buffer, 0, len * 4);
	
	//TODO can mute/unmute channels here
	readPulse(&mix_buffer[0], len, &pulse1);
	readPulse(&mix_buffer[len], len, &pulse2);
	readTriangle(&mix_buffer[2*len], len, &triangle);
	readNoise(&mix_buffer[3*len], len, &noise);
	
	//do mixing
	mixAudio(buffer, mix_buffer, len);
	
	//release memory
	//free(mix_buffer);
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	update_DAC_BUFFER(&DAC_BUFFER[0], DAC_BUFFER_LEN/2);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	update_DAC_BUFFER(&DAC_BUFFER[DAC_BUFFER_LEN/2], DAC_BUFFER_LEN/2);
}

void initRAM(uint16_t init_addr, uint16_t play_addr) {
	memset(pRAM, 0, 0x0800);
	memset(pSRAM, 0, 0x2000);
	memset(pExRAM, 0, 0x1000);
	pStack = pRAM + 0x100;

	pExRAM[0x00] = 0x20;						//JSR
	memcpy(&pExRAM[0x01],&init_addr,2);	//Init Address
	pExRAM[0x03] = 0xF2;						//JAM
	pExRAM[0x04] = 0x20;						//JSR
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

void initInstruction(void) {
	//do init instructions
	while (regPC != 0x5003) { //until PC reaches after INIT
		//execute next instruction
		emulate6502(nCPUCycle + 1);
		if (regPC < 0x5000 || (regPC > 0x5009 && regPC < 0x8000)) {
			led_bug = 1;
			break;
		}
	}
}

void initSong(uint8_t song_num) {
	regPC = 0x5000;
	regA = (song_num == 0 ? 1 : song_num) - 1; //safety again
	regX = bPALMode;
	regY = 0; //bCleanAXY ? 0 : 0xCD;
	regSP = 0xFF;
	//if(bCleanAXY)
	//	regP = 0x04;
	bCPUJammed = 0;

	nCPUCycle = nAPUCycle = 0;

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
	
	//apparently songs can be switched without calling INIT instructions
	//but let's just do what the wiki says
	initInstruction();
}


//========================//
// 	TONY IMPLEMENT THESE 	//
//========================//

// Load the SD card data into pROM_Full
// assume no bank switching for now and put the byte from 0x80 of NSF to load_addr - 0x8000 position of pROM_Full and so on
void load_ROM(void) {
	//reset
	memset(pROM_Full, 0, 0x8000);
	
	//Comment out these lines and do your thing
	int idx = 0xbdc4;
	for (uint32_t i=0; i<16956; ++i) {
		pROM_Full[idx - 0x8000 + i] = NSF_ARRAY[i];
	}
	
}

// Call this function after finishing reading nsf file
void load_NSF_data(void) {
	//load NSF data into ROM
	load_ROM();
	
	//init RAM stuff
	//comment out these 2 lines and instead use the NSF data
	uint16_t init_addr = 0xbe34;
	uint16_t play_addr = 0xf2d0;
	initRAM(init_addr, play_addr);
	
	//init the first song
	//change the default values to NSF data (i may group these variables to one object later, for now just use it)
	song_select = 1;
	num_songs = 18;
	initSong(song_select);
	
	//specify that song is loaded and can play
	song_loaded = 1;
}


//========================//
// 	TONY IMPLEMENT ABOVE 	//
//========================//


/**
	* Define callbacks for buttons here
	* Then remember to add callbacks in button.c
	*/
void SW5_pressed_callback(void) {
	if (!song_loaded) return;
	song_select = (song_select + 1) % num_songs;
	initSong(song_select);
}
void SW6_pressed_callback(void) {
	load_NSF_data();
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
	initTriangleLookup();
	initButtons();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)DAC_BUFFER, DAC_BUFFER_LEN, DAC_ALIGN_12B_R);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			
		//Wave generation thread
		static uint32_t last_wav_ticks = 0;

		//TODO update time = frame rate
		//TODO trying at 250Hz
		if (HAL_GetTick() - last_wav_ticks >= 4) {
			last_wav_ticks = HAL_GetTick();
			
			static uint8_t wav_counter = 0;
			
			//do frame clocking at 240Hz
			clockFrame();
			
			//do next frame at 60 Hz
			if (wav_counter == 0)
			{
				if(song_loaded && led_bug != 1) {
					while (!(regPC > 0x5FFF)) { //until PC reaches PLAY instructions
						//execute next instruction
						emulate6502(nCPUCycle + 1);
						if (regPC < 0x5000 || (regPC > 0x5009 && regPC < 0x8000) || (bCPUJammed == 1)) {
							led_bug = 1;
							break;
						}
					}
					while (regPC != 0x5007) { //until PC finishes PLAY instructions
						//execute next instruction
						emulate6502(nCPUCycle + 1);
						if (regPC < 0x5000 || (regPC > 0x5009 && regPC < 0x8000) || (bCPUJammed == 1)) {
							led_bug = 1;
							break;
						}
					}
				}
			}
			
			//update
			wav_counter = (wav_counter + 1) % 4;
			
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
		if (HAL_GetTick() - last_led_ticks >= 250) {
			last_led_ticks = HAL_GetTick();
			HAL_GPIO_TogglePin(LED1);
		}
		static uint32_t last_led2_ticks = 0;
		if (HAL_GetTick() - last_led2_ticks >= 10) {	
			HAL_GPIO_WritePin(LED2, (GPIO_PinState)(!(led_bug & 0x01)));
			HAL_GPIO_WritePin(LED3, (GPIO_PinState)(!(led_bug & 0x02)));
		}
		
		//button thread
		static uint32_t last_button_ticks = 0;
		if (HAL_GetTick() - last_button_ticks >= 50) {
			last_button_ticks = HAL_GetTick();
			updateButtons();
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
	
	//=============================================================================//
	// REMEMBER TO CHANGE PERIOD TO (47 * TIMER_RATE) - 1 AFTER CONFIG WITH CUBEMX //
	//=============================================================================//
	
  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (47 * TIMER_RATE) - 1;
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
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Button1_Pin */
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Button2_Pin Button3_Pin Button4_Pin Button5_Pin
                           Button6_Pin */
  GPIO_InitStruct.Pin = Button2_Pin|Button3_Pin|Button4_Pin|Button5_Pin
                          |Button6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
