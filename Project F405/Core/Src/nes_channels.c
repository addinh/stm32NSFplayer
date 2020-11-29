#include "nes_channels.h"

#define PULSE_MIX_SCALE 			0.00752
#define TRIANGLE_MIX_SCALE 		0.00851/2
#define NOISE_MIX_SCALE 			0.00494

//==========================================//
//							PULSE CHANNEL								//
//==========================================//
uint8_t pulseLookup[4][8] = {
	{0, 1, 0, 0, 0, 0, 0, 0},
	{0, 1, 1, 0, 0, 0, 0, 0},
	{0, 1, 1, 1, 1, 0, 0, 0},
	{1, 0, 0, 1, 1, 1, 1, 1}
};
void updatePulse(PulseChannel *pulse) {
	//volume -> amplitude
	pulse->wave.amplitude = (uint16_t)(pulse->volume * 4095 * PULSE_MIX_SCALE) * pulse->wave.enable;
	
	//timer -> period
	// f = CPU / (16 * (t + 1))
	pulse->wave.period = TIMER_DOUBLE_SCALER * (((uint16_t)pulse->timer_hi << 8) + pulse->timer_lo + 1);
	if (pulse->wave.period < 9) {
		pulse->wave.amplitude = 0;
	}
	
	//counter check
	if (pulse->wave.counter >= pulse->wave.period) pulse->wave.counter = pulse->wave.counter % pulse->wave.period;
}

void setPulseTimer(PulseChannel *pulse, uint16_t timer) {
	pulse->timer_lo = (uint8_t)(timer & 0xFF);
	pulse->timer_hi = (uint8_t)((timer >> 8) & 0x07);
}

void readPulse(uint16_t* buffer, const uint16_t buffer_len, PulseChannel* pulse) {
	if (pulse->wave.period == 0 || pulse->wave.amplitude == 0) {
		//channel not fully initialized
		//or channel is muted
		return;
	}
	for (uint16_t i = 0; i < buffer_len; ++i) {
		if (pulseLookup[pulse->duty][pulse->wave.pulse_counter]) {
			//REMEMBER TO RESET BUFFER TO 0
			buffer[i] += pulse->wave.amplitude;
		}
		
		//every 2*[period] CPU cycles, [pulse_counter] goes up by 1
		pulse->wave.counter = (pulse->wave.counter + TIMER_HALF_PRESCALER) % pulse->wave.period;
		if (pulse->wave.counter < TIMER_HALF_PRESCALER) {
			pulse->wave.pulse_counter = (pulse->wave.pulse_counter + 1) % 8;
		}
	}
}


//==========================================//
//						TRIANGLE CHANNEL							//
//==========================================//
void updateTriangle(TriangleChannel *tri) {
	//timer -> period
	// f = CPU / (32 * (t + 1))
	tri->wave.period = (((uint16_t)tri->timer_hi << 8) + tri->timer_lo + 1);
	
	//counter check
	if (tri->wave.counter >= tri->wave.period) tri->wave.counter = tri->wave.counter % tri->wave.period;
}

/*const uint16_t triangleLookup[32] = {
	4095, 3822, 3549, 3276, 3003, 2730, 2457, 2184, 1911, 1638, 1365, 1092, 819, 546, 273, 0,
	0, 273, 546, 829, 1092, 1365, 1638, 1911, 2184, 2457, 2730, 3003, 3276, 3549, 3822, 4095
};*/
uint16_t triangleLookupScaled[32] = { 0 };
void initTriangleLookup(void) {
	for (uint8_t i = 0; i < 32; ++i) {
		//j goes from 15 to 0 then 0 to 15
		uint8_t j = (i < 16) ? (15 - i) : (i - 16);
		triangleLookupScaled[i] = (uint16_t)(j * 4095 * TRIANGLE_MIX_SCALE);
	}
}
void readTriangle(uint16_t* buffer, const uint16_t buffer_len, TriangleChannel* tri) {
	if (tri->wave.period == 0 || tri->wave.period == 1) {
		//channel not fully initialized
		//or channel is muted
		return;
	}
	for (uint16_t i = 0; i < buffer_len; ++i) {
		const uint16_t amplitude = triangleLookupScaled[tri->wave.amplitude_counter] * tri->wave.enable;
		//REMEMBER TO RESET BUFFER TO 0
		buffer[i] += amplitude;
		
		//every [period] CPU cycles, [amplitude_counter] goes up by 1
		tri->wave.counter = (tri->wave.counter + TIMER_PRESCALER) % tri->wave.period;
		if (tri->wave.counter < TIMER_PRESCALER) {
			tri->wave.amplitude_counter = (tri->wave.amplitude_counter + 1) % 32;
		}
	}
}

void setTriangleTimer(TriangleChannel *tri, uint16_t timer) {
	tri->timer_lo = (uint8_t)(timer & 0xFF);
	tri->timer_hi = (uint8_t)((timer >> 8) & 0x07);
}


//==========================================//
//							NOISE CHANNEL								//
//==========================================//
const uint16_t NTSC_noise_period[] = {
	4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};
void updateNoise(NoiseChannel *noisy) {
	//volume -> amplitude
	noisy->wave.amplitude = (uint16_t)(noisy->volume * 4095 * NOISE_MIX_SCALE) * noisy->wave.enable;
	
	//get period from lookup table
	noisy->wave.period = TIMER_DOUBLE_SCALER * NTSC_noise_period[noisy->period_index];
	
	//counter check
	if (noisy->wave.counter >= noisy->wave.period) noisy->wave.counter = noisy->wave.counter % noisy->wave.period;
}

#define BIT(X, BIT_POS) (((X) >> BIT_POS) & 0x01)   
void readNoise(uint16_t* buffer, const uint16_t buffer_len, NoiseChannel* noisy) {
	if (noisy->wave.period == 0 || noisy->wave.amplitude == 0) {
		//channel not fully initialized
		//or channel is muted
		return;
	}
	for (uint16_t i = 0; i < buffer_len; ++i) {
		if (BIT(noisy->shift_register, 0) == 1) {
			//REMEMBER TO RESET BUFFER TO 0
			buffer[i] += noisy->wave.amplitude;
		}
		
		//every 2*[period] CPU cycles, [shift_register] is updated
		noisy->wave.counter = (noisy->wave.counter + TIMER_HALF_PRESCALER) % noisy->wave.period;
		if (noisy->wave.counter < TIMER_HALF_PRESCALER) {
			uint8_t feedback = BIT(noisy->shift_register, 0) ^ BIT(noisy->shift_register, (noisy->mode ? 6 : 1));
			noisy->shift_register >>= 1;
			noisy->shift_register &= 16383; //set bit 14 to 0
			noisy->shift_register |= feedback << 14; //OR bit 14 with feedback
		}
	}
}
