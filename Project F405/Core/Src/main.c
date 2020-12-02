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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "nes_channels.h"
#include "emulator6502.h"
#include "button.h"

//Tony's include library
#include "lcd.h"
#include "nsf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED1 GPIOC, GPIO_PIN_1
#define LED2 GPIOC, GPIO_PIN_2
#define LED3 GPIOC, GPIO_PIN_3

//#define SAFE_TESTING
#ifdef SAFE_TESTING
	#include "nsf_array.h"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
#define DAC_BUFFER_LEN 256
uint16_t DAC_BUFFER[DAC_BUFFER_LEN + 16] = {0};

//music player variables
//uint8_t song_loaded = 0;
//uint8_t song_select = 0;
//uint8_t num_songs = 18;
//uint8_t led_bug = 0;
struct {
	uint8_t nsf_loaded;
	uint8_t song_loaded;
	uint8_t song_select;
	uint8_t num_songs;
	uint8_t paused;
} musicPlayer;

volatile uint8_t update_flag = 0;
//volatile uint8_t song_changing = 0;

//NES channels
PulseChannel pulse1 = {
	.volume = 0,
	.sweep_complement = 1,
};
	
PulseChannel pulse2 = {
	.volume = 0,
	.sweep_complement = 0,
};
	
TriangleChannel triangle = {
	.timer_lo = 0,
};
	
NoiseChannel noise = {
	.shift_register = 1,
	.volume = 0,
};

//sd card variables
extern char SDPath[4];  
extern FATFS SDFatFS;  
extern FIL SDFile; 

FRESULT res;                                         
uint32_t byteswritten, bytesread;
nsf_file file;
char file_name_list[10][40];

#define max_song 10//for easier implementation without using daynamic array
#define total_channel 4
extern TFT_Select_list selected;
extern TFT_Select_list_detail selected_detail;
int file_count =0;
int other_stuff=0;
int sd_card_inserted_flag =0;
int channel_enabled[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
FRESULT scan_files (char* path);
void scan_nsf_file(char * file_name);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define AUDIO_VOLUME 0.3f

void mixAudio(uint16_t *buffer, uint8_t *mix_buffer, uint16_t len) {
	//perform mixing on a buffer 4 times the length of (half) output to DAC buffer
	//overall scaling done here
	//note that len should be the length of the output buffer aka 1/4 length of buffer
	#pragma unroll 8
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

//helper for force mute/unmute all channels
void forceMute(uint8_t muted) {
	pulse1.channel_muted = muted;
	pulse2.channel_muted = muted;
	triangle.channel_muted = muted;
	noise.channel_muted = muted;
}	

//going safe method here
uint8_t mix_buffer[DAC_BUFFER_LEN * 4 / 2] = {0};

// DMA INTERRUPTS
void update_DAC_BUFFER(uint16_t *buffer, uint16_t len) {
	//if changing song, return
	//if (song_changing) return;
	
	//create a temp array for mixing
	//uint8_t* mix_buffer = (uint8_t*)malloc(len * 4);
	memset(mix_buffer, 0, len * 4);
	
	if (!musicPlayer.paused) {
		readPulse(&mix_buffer[0], len, &pulse1);
		readPulse(&mix_buffer[len], len, &pulse2);
		readTriangle(&mix_buffer[2*len], len, &triangle);
		readNoise(&mix_buffer[3*len], len, &noise);
	}
	
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
			musicPlayer.paused = 1;
			break;
		}
	}
}

void initSong(void) {
	//set flag
	//__disable_irq();
	//song_changing = 1;
	//fastmemset(DAC_BUFFER, 0, DAC_BUFFER_LEN * 2);
	uint8_t song_num = musicPlayer.song_select;
	if (song_num == 0) {
		musicPlayer.song_loaded = 0;
		forceMute(1);
		return;
	}
	else {
		musicPlayer.song_loaded = 1;
		forceMute(0);
	}
	
	regPC = 0x5000;
	regA = song_num - 1;
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
		WriteMemory_ExRAM(0x5FF6 + i, nBankswitchInitValues[i]);
	}
	
	//apparently songs can be switched without calling INIT instructions
	//but let's just do what the wiki says
	initInstruction();
	
	//reset flag
	//song_changing = 0;
	//__enable_irq();
}


//========================//
// 	TONY IMPLEMENT THESE 	//
//========================//

// Load the SD card data into pROM_Full
// assume no bank switching for now and put the byte from 0x80 of NSF to load_addr - 0x8000 position of pROM_Full and so on
void load_ROM() {
	//reset
	//memset(pROM_Full, 0, 0x8000);
	
	//Comment out these lines and do your thing
	#ifdef SAFE_TESTING
	int idx = 0xbdc4;
	for (uint32_t i=0; i<16956; ++i) {
		pROM_Full[idx - 0x8000 + i] = NSF_ARRAY[i];
	}
	#endif
	
//	int idx = load_address;
//	for (uint32_t i=0; i<file_byte; ++i) {
//		pROM_Full[idx - 0x8000 + i] = music_data[i];
//	}
	
}

// Call this function after finishing reading nsf file
void load_NSF_data(nsf_file* file) {
	//load NSF data into ROM
	//int load_address = (int16_t)file.format.load_address_orgin[1]<<8 | file.format.load_address_orgin[0];
	//load_ROM();
	
	//init RAM stuff
	//comment out these 2 lines and instead use the NSF data
//	uint16_t init_addr = 0xbe34;
//	uint16_t play_addr = 0xf2d0;
//	initRAM(init_addr, play_addr);
	uint16_t init_addr = (int16_t)file->format.init_address_orgin[1]<<8 | file->format.init_address_orgin[0];
	uint16_t play_addr = (int16_t)file->format.play_address_orgin[1]<<8 | file->format.play_address_orgin[0];
	initRAM(init_addr, play_addr);
	
	//bankswitching setup
	uint8_t use_bankswitch = 0;
	for (uint8_t i=0; i<8; ++i) use_bankswitch += file->format.Bankswitch[i];
	if (use_bankswitch) {
		for (uint8_t i=0; i<10; ++i) {
			if (i >= 2) {
				nBankswitchInitValues[i] = file->format.Bankswitch[i-2];
			}
			else nBankswitchInitValues[i] = i;
		}
	}
	else {
		for (uint8_t i=0; i<10; ++i) {
			//TODO: according to famitracker, pROM[i] points to pROM_Full[0x0...] for banks less than load address?
			if (i >= 2) nBankswitchInitValues[i] = i-2;
			else nBankswitchInitValues[i] = i;
		}
	}
	
	//init the first song
	//change the default values to NSF data (i may group these variables to one object later, for now just use it)
	musicPlayer.song_select = 0;//file->format.starting_song;
	musicPlayer.num_songs = file->format.total_song;
	initSong();
	
	//unpause
	musicPlayer.nsf_loaded = 1;
	musicPlayer.paused = 0;
}


//========================//
// 	TONY IMPLEMENT ABOVE 	//
//========================//

//Frame Counter Timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim != &htim2) return;
	update_flag = 1;
}


/**
	* Define callbacks for buttons here
	* Then remember to add callbacks in button.c
	*/
void SW5_pressed_callback(void) {
	//TODO song select left/right
	if (!musicPlayer.nsf_loaded) return;
	switch(selected_detail){
			case TFT_PAUSE:
				musicPlayer.paused ^= 1;
				break;
			case TFT_P1:
				pulse1.channel_muted ^=1;
			break;
			case TFT_P2:
				pulse2.channel_muted ^=1;
			break;
			case TFT_TR:
				triangle.channel_muted ^=1;
			break;
			case TFT_NS:
				noise.channel_muted ^=1;
			break;
			case TFT_SONG:
				
			default:
				break;
		}
}
//uint8_t first = 1;
void SW2_pressed_callback(void) {
	if (!musicPlayer.nsf_loaded) {
		#ifdef SAFE_TESTING
			load_ROM();
			uint16_t init_addr = 0xbe34;
			uint16_t play_addr = 0xf2d0;
			initRAM(init_addr, play_addr);
			song_select = 2;
			num_songs = 18;
			initSong(song_select);
			//if (first) {
			//	initInstruction();
			//	first = 0;
			//}
			song_loaded = 1;
		#else
			scan_nsf_file(file_name_list[selected-1]);
		#endif
		selected_detail = TFT_SONG;
	}
	else{
		musicPlayer.nsf_loaded = 0;
		//memset(DAC_BUFFER,0,DAC_BUFFER_LEN*2);
		tft_update(0);
		forceMute(1);
	}
}

void SW4_pressed_callback(void) {
	if(!musicPlayer.nsf_loaded) {
		selected = selected +1;
		if(selected >file_count+other_stuff){
			//if(file_count+other_stuff==0)
			//	selected = 0;
			//else if(other_stuff==0)
				selected = 1;
			//else if(selected >file_count)
			//	selected = max_song+1;
			//else
			//	selected=1;
		}
	}
	else{
		switch(selected_detail){
			case TFT_SONG:
				selected_detail=TFT_P1;
				break;
			case TFT_PAUSE:
				selected_detail=TFT_SONG;
				break;
			case TFT_P1:
			case TFT_P2:
			case TFT_TR:
			case TFT_NS:
				selected_detail=TFT_PAUSE;
				break;
			default:
				break;
		}
	}
}

void SW6_pressed_callback(void) {
	if(!musicPlayer.nsf_loaded) {
		selected = selected - 1;
		if(selected < 1){
			//if(file_count+other_stuff==0)
			//	selected = 0;
			//else if(other_stuff==0)
				selected = file_count;
			//else if(selected < 1)
			//	selected = max_song+1;
			//else
			//	selected=1;
		}
	}
	else{
		switch(selected_detail){
			case TFT_SONG:
				selected_detail=TFT_PAUSE;
				break;
			case TFT_PAUSE:
				selected_detail=TFT_P1;
				break;
			case TFT_P1:
			case TFT_P2:
			case TFT_TR:
			case TFT_NS:
				selected_detail=TFT_SONG;
				break;
			default:
				break;
		}
	}
}

void SW3_pressed_callback(void) {
	if (!musicPlayer.nsf_loaded) return;
	switch(selected_detail){
			case TFT_SONG:
				if (musicPlayer.song_select == 0) musicPlayer.song_select = musicPlayer.num_songs + 1;
				musicPlayer.song_select = (musicPlayer.song_select - 1) % (musicPlayer.num_songs + 1);
				initSong();
				break;
			case TFT_PAUSE:
				break;
			case TFT_P1:
				selected_detail = TFT_NS;
			break;
			case TFT_P2:
				selected_detail = TFT_P1;
			break;
			case TFT_TR:
				selected_detail = TFT_P2;
			break;
			case TFT_NS:
				selected_detail = TFT_TR;
			break;
			default:
				break;
		}
}

void SW7_pressed_callback(void){
	if (!musicPlayer.nsf_loaded) return;
	switch(selected_detail){
			case TFT_SONG:
				musicPlayer.song_select = (musicPlayer.song_select + 1) % (musicPlayer.num_songs + 1);
				initSong();
				break;
			case TFT_PAUSE:
				break;
			case TFT_P1:
				selected_detail = TFT_P2;
			break;
			case TFT_P2:
				selected_detail = TFT_TR;
			break;
			case TFT_TR:
				selected_detail = TFT_NS;
			break;
			case TFT_NS:
				selected_detail = TFT_P1;
			break;
			default:
				break;
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
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	initTriangleLookup();
	initButtons();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim6);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)DAC_BUFFER, DAC_BUFFER_LEN, DAC_ALIGN_12B_R);
	//TFT init and sd card mount and get the file name
	tft_init(PIN_ON_TOP, WHITE, BLACK, RED, GREEN);
	char buff[10];
	//HAL_Delay(1000);
	res= f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
	sd_card_inserted_flag=1;
//	if(res != FR_OK)
//			sd_card_inserted_flag=0;
//	else
//		sd_card_inserted_flag=1;
	strcpy(buff, "/");	
	res = scan_files(buff);
//	for(uint32_t i=0; i<total_channel;i++){
//		channel_enabled[i]= 1;
//	}

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//TFT_print thread
		//Wave generation thread
		static uint32_t last_tft_ticks = 0;

		//TODO update time = frame rate
		//TODO trying at 250Hz
		if (HAL_GetTick() - last_tft_ticks >= 50) {
			last_tft_ticks = HAL_GetTick();
			
		//check for sd card
//		if(sd_card_inserted_flag==0){
//		res= f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
//			if(res != FR_OK)
//				sd_card_inserted_flag=0;
//			else
//				sd_card_inserted_flag=1;
//		}
//		if(sd_card_inserted_flag==1){
//		res = f_stat("SUPERM~1.NSF",NULL);
//			if(res != FR_OK)
//				sd_card_inserted_flag=0;
//			else
//				sd_card_inserted_flag=1;
//		}
		
		
		if(sd_card_inserted_flag==1){
			if(!musicPlayer.nsf_loaded){
				tft_prints(0,3,"Files:");
				for (uint32_t i =0; i< file_count; i ++){
					custom_tft_prints(0,4+i,i+1,0,file_name_list[i]);
				}
			}
			else{
				char buffer[10];
				tft_prints(0,0,"Name: ");
				tft_prints(7,0,file.format.name);
				tft_prints(0,1,"Artist: ");
				tft_prints(9,1,file.format.artist);
				tft_prints(0,2,"Copyright: ");
				tft_prints(12,2,file.format.copy_right);
				custom_tft_prints(0,3,TFT_SONG,1,"current song: ");
				tft_prints(15,3,"%d/%d",musicPlayer.song_select,musicPlayer.num_songs);
				
				
				tft_prints(0,4,"Channels: ");
				if(pulse1.channel_muted == 0){
					tft_set_text_color(BLUE);
					custom_tft_prints(10,4,TFT_P1,1,"ch_1");
					tft_set_text_color(BLACK);
				}
				else
					custom_tft_prints(10,4,TFT_P1,1,"ch_1");
				if(pulse2.channel_muted == 0){
					tft_set_text_color(BLUE);
					custom_tft_prints(17,4,TFT_P2,1,"ch_2");
					tft_set_text_color(BLACK);
				}
				else
					custom_tft_prints(17,4,TFT_P2,1,"ch_2");
				if(triangle.channel_muted == 0){
					tft_set_text_color(BLUE);
					custom_tft_prints(24,4,TFT_TR,1,"ch_3");
					tft_set_text_color(BLACK);
				}
				else
					custom_tft_prints(24,4,TFT_TR,1,"ch_3");
				if(noise.channel_muted  == 0){
					tft_set_text_color(BLUE);
					custom_tft_prints(31,4,TFT_NS,1,"ch_4");
					tft_set_text_color(BLACK);
				}
				else
					custom_tft_prints(31,4,TFT_NS,1,"ch_4");
				
				custom_tft_prints(0,5,TFT_PAUSE,1,"PAUSED: ");
				tft_prints(10,5,"%d",(int)musicPlayer.paused);
	
				
		}
	 }
		else{
			tft_prints(0,3,"SD card is not inserted");
		}
		tft_update(0);
	 }
		//Wave generation thread
		static uint32_t last_wav_ticks = 0;

		//TODO update time = frame rate
		//TODO trying at 250Hz
		if (HAL_GetTick() - last_wav_ticks >= 1) {
			last_wav_ticks = HAL_GetTick();
			
			if (update_flag) {
				static uint8_t wav_counter = 0;
			
				//do frame clocking at 240Hz
				clockFrame();
			
				//do next frame at 60 Hz
				if (wav_counter == 0)
				{
					if(musicPlayer.song_loaded && !musicPlayer.paused) {
						while (!(regPC > 0x5FFF)) { //until PC reaches PLAY instructions
							//execute next instruction
							emulate6502(nCPUCycle + 1);
							if (regPC < 0x5000 || (regPC > 0x5009 && regPC < 0x8000) || (bCPUJammed == 1)) {
								musicPlayer.paused = 1;
								break;
							}
						}
						while (regPC != 0x5007) { //until PC finishes PLAY instructions
							//execute next instruction
							emulate6502(nCPUCycle + 1);
							if (regPC < 0x5000 || (regPC > 0x5009 && regPC < 0x8000) || (bCPUJammed == 1)) {
								musicPlayer.paused = 1;
								break;
							}
						}
					}
				}
			
				//update
				wav_counter = (wav_counter + 1) % 4;
			
				//reset flag
				update_flag = 0;
			}
		}
		
		//LED thread
		static uint32_t last_led_ticks = 0;
		if (HAL_GetTick() - last_led_ticks >= 250) {
			last_led_ticks = HAL_GetTick();
			HAL_GPIO_TogglePin(LED3);
		}
		static uint32_t last_led2_ticks = 0;
		if (HAL_GetTick() - last_led2_ticks >= 10) {	
			HAL_GPIO_WritePin(LED2, (GPIO_PinState)(!(musicPlayer.paused & 0x01)));
			HAL_GPIO_WritePin(LED1, (GPIO_PinState)(!(musicPlayer.paused & 0x02)));
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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 20;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 349999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button2_Pin Button3_Pin Button4_Pin Button5_Pin
                           Button6_Pin */
  GPIO_InitStruct.Pin = Button2_Pin|Button3_Pin|Button4_Pin|Button5_Pin
                          |Button6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
FRESULT scan_files (char* path)
{
    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;

		int count=0;
    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                res = scan_files(path);                    /* Enter the directory */
                if (res != FR_OK) break;
                path[i] = 0;
            } else {                                       /* It is a file. */
							  sprintf(file_name_list[count],"%s",fno.fname);
							count = count +1;
            }
        }
        f_closedir(&dir);
				file_count =count;
    }

    return res;
}

void scan_nsf_file(char * file_name){
	  res =f_open(&SDFile, file_name,  FA_READ);
		if(res != FR_OK)
			tft_prints(0, 15, "%d",res);
		int file_byte = f_size(&SDFile);
		memset(file.text,0,128);
		res = f_read(&SDFile, file.text, 128, (UINT*)&bytesread);
		if((bytesread == 0) || (res != FR_OK))
		{
			tft_prints(0, 18, "%d",res);
		}
		else 
		{
			tft_prints(0, 18, "Read ok",2);
		}
		tft_update(0);
		tft_prints(0, 19, "%d",file_byte);
		int file_byte_left = file_byte-128;
		int idx = (int16_t)file.format.load_address_orgin[1]<<8 | file.format.load_address_orgin[0];
		
		//program only supports reading up to 9KB music data
		//if file size too big, cancel
		//TODO what happens if cancel? tft print something?
		if (file_byte_left > MAX_ROM_SIZE) return;
		
		memset(pROM_Full,0,MAX_ROM_SIZE);
		
		#define CHUNK_SIZE 256
		
		while(file_byte_left>0){
			if(file_byte_left >= CHUNK_SIZE){
				uint8_t part_byte[CHUNK_SIZE];
				res = f_read(&SDFile, part_byte, CHUNK_SIZE, (UINT*)&bytesread);
				for (uint32_t i=0; i<CHUNK_SIZE; ++i) {
					pROM_Full[idx - 0x8000 +file_byte-128-file_byte_left+i] = part_byte[i];
	      }
				file_byte_left = file_byte_left-CHUNK_SIZE;
			}
			else{
				uint8_t part_byte[file_byte_left];
				res = f_read(&SDFile, part_byte, file_byte_left, (UINT*)&bytesread);
				for (uint32_t i=0; i<file_byte_left; ++i) {
					pROM_Full[idx - 0x8000+file_byte-128-file_byte_left+i] = part_byte[i];
	      }
				file_byte_left = file_byte_left-file_byte_left;
			}
			
		}
		if(!f_eof(&SDFile))
			tft_prints(0, 19, "Not eof!");
		f_close(&SDFile);
		tft_update(0);
	  load_NSF_data(&file);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
