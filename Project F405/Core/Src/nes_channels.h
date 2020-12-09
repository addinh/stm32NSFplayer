#ifndef __NES_CHANNELS_H
#define __NES_CHANNELS_H

#include "stdint.h"
#include "stdio.h"
#include "string.h"

#pragma anon_unions


//this is how slow the TIM6 channel is compared to the intended NTSC CPU freq (~1.8 MHz)
//alternatively, this is how many NTSC CPU cycles have passed between each DAC sample
#define TIMER_RATE 4		//2 works for playing 1 song, needs to solve INIT bottleneck [stuck in DMA IRQ loop???]
#define TIMER_PRESCALER TIMER_RATE
#define TIMER_HALF_PRESCALER (TIMER_RATE/2)
#define TIMER_DOUBLE_SCALER (TIMER_RATE == 1 ? 2 : 1)

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
			uint8_t const_volume_flag : 1;
			uint8_t length_counter_halt : 1;
			uint8_t duty : 2;
			
			uint8_t sweep_shift : 3;
			uint8_t sweep_negate : 1;
			uint8_t sweep_period : 3;
			uint8_t sweep_enable : 1;
			
			uint8_t timer_lo;
			
			uint8_t timer_hi : 3;
			uint8_t length_counter : 5;
		};
		struct {
			uint8_t registers[4];
		};
	};
	//wave generation
	struct {
		uint8_t enable;
		uint8_t muted;
		uint8_t volume;
		uint16_t amplitude;
		uint16_t period;
		uint16_t counter;
		uint16_t pulse_counter;
		uint8_t envelope_counter;
		uint8_t sweep_counter;
		uint8_t length_counter;
	} wave;
	uint8_t sweep_complement;
	uint8_t channel_muted;
} PulseChannel;

//2 pulse channel instances
extern PulseChannel pulse1, pulse2;

//update the wave generation parameters according to registers
inline void updatePulse(PulseChannel *pulse, uint8_t reg_num);

//update muted status of channel (due to low period or sweep)
inline void checkMutedPulse(PulseChannel *pulse);

//update the wave generation parameters due to frame counter
inline void clockPulse(PulseChannel *pulse, uint8_t step);

//read the pulse wave into a buffer
inline void readPulse(uint8_t* buffer, const uint16_t buffer_len, PulseChannel* pulse);

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
			uint8_t counter_flag : 1; //is also length_counter_halt

			uint8_t UNUSED;
			
			uint8_t timer_lo;
			
			uint8_t timer_hi : 3;
			uint8_t length_counter : 5;
		};
		struct {
			uint8_t registers[4];
		};
	};
	//wave generation
	struct {
		uint8_t enable;
		uint16_t period;
		uint16_t counter;
		uint16_t amplitude_counter;
		uint8_t length_counter;
		uint8_t linear_counter;
		uint8_t linear_counter_flag;
	} wave;
	uint8_t channel_muted;
} TriangleChannel;

//1 triangle channel instance
extern TriangleChannel triangle;

//call this in init
void initTriangleLookup(void);

//update the wave generation parameters according to registers
inline void updateTriangle(TriangleChannel *tri, uint8_t reg_num);

//update the wave generation parameters due to frame counter
inline void clockTriangle(TriangleChannel *tri, uint8_t step);

//read the triangle wave into a buffer
inline void readTriangle(uint8_t* buffer, const uint16_t buffer_len, TriangleChannel* tri);

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
			uint8_t UUUUUUUU;
			uint8_t MUUUPPPP;
			uint8_t LLLLLUUU;
		};
		struct {
			uint8_t volume : 4;
			uint8_t const_volume_flag : 1;
			uint8_t length_counter_halt : 1;
			uint8_t unused : 2;
			
			uint8_t UNUSED;
			
			uint8_t period_index : 4;
			uint8_t unused2 : 3;
			uint8_t mode : 1;
			
			uint8_t unused3 : 3;
			uint8_t length_counter : 5;
		};
		struct {
			uint8_t registers[4];
		};
	};
	//wave generation
	struct {
		uint8_t enable;
		uint8_t volume;
		uint16_t amplitude;
		uint16_t period;
		uint16_t counter;
		uint8_t envelope_counter;
		uint8_t length_counter;
	} wave;
	uint8_t channel_muted;
} NoiseChannel;

//1 noise channel instance
extern NoiseChannel noise;

//update the wave generation parameters according to registers
inline void updateNoise(NoiseChannel *noisy, uint8_t reg_num);

//update the wave generation parameters due to frame counter
inline void clockNoise(NoiseChannel *noisy, uint8_t step);

//read the pulse wave into a buffer
inline void readNoise(uint8_t* buffer, const uint16_t buffer_len, NoiseChannel* noisy);


// FrameCounter
// Call this at 240 Hz
inline void clockFrame(void);

#endif
