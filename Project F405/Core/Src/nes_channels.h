#ifndef __NES_CHANNELS_H
#define __NES_CHANNELS_H

#include "stdint.h"

#pragma anon_unions

//this is how slow the TIM6 channel is compared to the intended NTSC CPU freq (~1.8 MHz)
//alternatively, this is how many NTSC CPU cycles have passed between each DAC sample
#define TIMER_PRESCALER 2
#define TIMER_HALF_PRESCALER 1
#define TIMER_DOUBLE_SCALER 1

//==========================================//
//							PULSE CHANNEL								//
//==========================================//
typedef struct {
	//register
	union {
		struct {
			uint8_t DDLCVVVV;
			uint8_t EPPPNSSS;
			uint8_t TTTTTTTT;
			uint8_t LLLLLTTT;
		};
		struct {
			uint8_t volume : 4;
			uint8_t envelope_flag : 1;
			uint8_t length_counter_halt : 1;
			uint8_t duty : 2;
			
			uint8_t shift : 3;
			uint8_t negate : 1;
			uint8_t sweep_period : 3;
			uint8_t sweep_enable : 1;
			
			uint8_t timer_lo;
			
			uint8_t timer_hi : 3;
			uint8_t length_counter : 5;
		};
	};
	//wave generation
	struct {
		uint8_t enable;
		uint16_t amplitude;
		uint16_t period;
		uint16_t counter;
		uint16_t pulse_counter;
	} wave;
} PulseChannel;

//2 pulse channel instances
static PulseChannel pulse1, pulse2;

//update the wave generation parameters according to registers
inline void updatePulse(PulseChannel *pulse);

//read the pulse wave into DMA buffer
inline void readPulse(uint16_t* buffer, const uint16_t buffer_len, PulseChannel* pulse);

//convenient setting timer for testing
inline void setPulseTimer(PulseChannel *pulse, uint16_t timer);


//==========================================//
//						TRIANGLE CHANNEL							//
//==========================================//
typedef struct {
	//register
	union {
		struct {
			uint8_t CRRRRRRR;
			uint8_t UUUUUUUU;
			uint8_t TTTTTTTT;
			uint8_t LLLLLTTT;
		};
		struct {
			uint8_t counter_reload : 7;
			uint8_t counter_flag : 1;

			uint8_t UNUSED;
			
			uint8_t timer_lo;
			
			uint8_t timer_hi : 3;
			uint8_t length_counter : 5;
		};
	};
	//wave generation
	struct {
		uint8_t enable;
		uint16_t period;
		uint16_t counter;
		uint16_t amplitude_counter;
	} wave;
} TriangleChannel;

//1 triangle channel instance
static TriangleChannel triangle;

//call this in init
void initTriangleLookup(void);

//update the wave generation parameters according to registers
inline void updateTriangle(TriangleChannel *tri);

//read the triangle wave into DMA buffer
inline void readTriangle(uint16_t* buffer, const uint16_t buffer_len, TriangleChannel* tri);

//convenient setting timer for testing
inline void setTriangleTimer(TriangleChannel *tri, uint16_t timer);


//==========================================//
//							NOISE CHANNEL								//
//==========================================//
typedef struct {
	uint16_t shift_register; //init this to 1
	//register
	union {
		struct {
			uint8_t UULCVVVV;
			uint8_t MUUUPPPP;
			uint8_t LLLLLUUU;
		};
		struct {
			uint8_t volume : 4;
			uint8_t envelope_flag : 1;
			uint8_t length_counter_halt : 1;
			uint8_t unused : 2;
			
			uint8_t period_index : 4;
			uint8_t unused2 : 3;
			uint8_t mode : 1;
			
			uint8_t length_counter : 5;
			uint8_t unused3 : 3;
		};
	};
	//wave generation
	struct {
		uint8_t enable;
		uint16_t amplitude;
		uint16_t period;
		uint16_t counter;
	} wave;
} NoiseChannel;

//1 noise channel instance
static NoiseChannel noise;

//update the wave generation parameters according to registers
inline void updateNoise(NoiseChannel *noisy);

//read the pulse wave into DMA buffer
inline void readNoise(uint16_t* buffer, const uint16_t buffer_len, NoiseChannel* noisy);

#endif
