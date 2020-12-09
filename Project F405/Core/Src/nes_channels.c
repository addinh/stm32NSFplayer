#include "nes_channels.h"
#include "emulator6502.h"

//additional scaling per channel
#define PULSE_MIX_SCALE 			1.0f
#define TRIANGLE_MIX_SCALE 		1.0f
#define NOISE_MIX_SCALE 			0.4f

//#define OPTIMIZE_SPEED

//buffer for unused pAPU registers just in case (DMC)
uint8_t pAPU_unused[0x20] = {0};

//length counter
const uint8_t LENGTH_COUNTER_LOOKUP[] = {10, 254, 20,  2, 40,  4, 80,  6, 160,  8, 60, 10, 14, 12, 26, 14, 
																				 12,  16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30};

//frame counter  (CLOCK_LOOKUP[mode][frame type][step]) (for mode 0, step 4 = step 0)
const uint8_t CLOCK_LOOKUP[2][2][5] = {{{1, 1, 1, 1, 1}, {0, 1, 0, 1, 0}},
																			 {{1, 1, 1, 0, 1}, {0, 1, 0, 0, 1}}};
struct {
	uint8_t mode;
	uint8_t interrupt_flag; //not implemented yet
	uint8_t clock_counter;
} frameCounter;	

void clockFrame(void) {
	clockPulse(&pulse1, frameCounter.clock_counter);
	clockPulse(&pulse2, frameCounter.clock_counter);
	clockTriangle(&triangle, frameCounter.clock_counter);
	clockNoise(&noise, frameCounter.clock_counter);
	
	frameCounter.clock_counter = (frameCounter.clock_counter + 1) % (frameCounter.mode ? 5 : 4);
}

/**
	* Functions handling read/write to "virtual" APU
	* Whenever registers are updated via write, call corresponding update() functions to update wave properties
	*
	*/
BYTE ReadMemory_pAPU(WORD a) {
	switch (a) {
		case 0x4000: case 0x4001: case 0x4002: case 0x4003: return pulse1.registers[a & 0x03];
		case 0x4004: case 0x4005: case 0x4006: case 0x4007: return pulse2.registers[a & 0x03];
		case 0x4008: case 0x4009: case 0x400A: case 0x400B: return triangle.registers[a & 0x03];
		case 0x400C: case 0x400D: case 0x400E: case 0x400F: return noise.registers[a & 0x03];
		
		case 0x4015:
		{
			BYTE ret = 0;
			if (pulse1.length_counter > 0) ret += 0x01;
			if (pulse2.length_counter > 0) ret += 0x02;
			if (triangle.length_counter > 0) ret += 0x04;
			if (noise.length_counter > 0) ret += 0x08;
			return ret;
		}
		
		case 0x4017:
			return (frameCounter.mode ? 0x80 : 0x00) + (frameCounter.interrupt_flag ? 0x40 : 0x00);

		default: return pAPU_unused[a & 0x1F];
	}
}


void WriteMemory_pAPU(WORD a, BYTE v) {
	switch (a) {
		case 0x4000: case 0x4001: case 0x4002: case 0x4003: 
			pulse1.registers[a & 0x03] = v;
			updatePulse(&pulse1, a & 0x03);
			break;
		case 0x4004: case 0x4005: case 0x4006: case 0x4007:
			pulse2.registers[a & 0x03] = v;
			updatePulse(&pulse2, a & 0x03);
			break;
		case 0x4008: case 0x4009: case 0x400A: case 0x400B:
			triangle.registers[a & 0x03] = v;
			updateTriangle(&triangle, a & 0x03);
			break;
		case 0x400C: case 0x400D: case 0x400E: case 0x400F:
			noise.registers[a & 0x03] = v;
			updateNoise(&noise, a & 0x03);
			break;
		
		case 0x4015:
			pulse1.wave.enable = (v & 0x01) >> 0;
			pulse2.wave.enable = (v & 0x02) >> 1;
			triangle.wave.enable = (v & 0x04) >> 2;
			noise.wave.enable = (v & 0x08) >> 3;
			break;
		
		case 0x4017:
			frameCounter.mode = (v & 0x80) >> 7;
			frameCounter.interrupt_flag = (v & 0x40) >> 6;
			frameCounter.clock_counter = 0;
			break;

		default: pAPU_unused[a & 0x1F] = v;
	}
}


//==========================================//
//							PULSE CHANNEL								//
//==========================================//
uint8_t PULSE_LOOKUP[4][8] = {
	{0, 1, 0, 0, 0, 0, 0, 0},
	{0, 1, 1, 0, 0, 0, 0, 0},
	{0, 1, 1, 1, 1, 0, 0, 0},
	{1, 0, 0, 1, 1, 1, 1, 1}
};
void updatePulse(PulseChannel *pulse, uint8_t reg_num) {
	//length counter update only if 4th register is updated
	if (reg_num == 4 - 1) {
		if (pulse->wave.enable)
			pulse->wave.length_counter = LENGTH_COUNTER_LOOKUP[pulse->length_counter];
		else 
			pulse->wave.length_counter = 0;
	}
	
	//if constant volume, set default volume as input volume
	if (pulse->const_volume_flag) pulse->wave.volume = pulse->volume;
	//else, set volume to 15 (if 4th register is updated)
	else if (reg_num == 4-1) pulse->wave.volume = 15;
	
	//timer -> period
	// f = CPU / (16 * (t + 1))
	//due to sweep, only update if 3rd or 4th registers are updated
	if (reg_num == 3 - 1 || reg_num == 4 - 1) {
		pulse->wave.period = TIMER_DOUBLE_SCALER * (((uint16_t)pulse->timer_hi << 8) + pulse->timer_lo + 1);
		checkMutedPulse(pulse);
	}
	
	//counter check
	if (pulse->wave.counter >= pulse->wave.period) pulse->wave.counter = pulse->wave.counter % pulse->wave.period;
}

void setPulseTimer(PulseChannel *pulse, uint16_t timer) {
	pulse->timer_lo = (uint8_t)(timer & 0xFF);
	pulse->timer_hi = (uint8_t)((timer >> 8) & 0x07);
}

//helper for doing sweep, returns the target period
uint16_t doSweep(uint16_t period, uint8_t shift, uint8_t negate, uint8_t complement) {
	uint16_t shifted = period >> shift;
	if (negate) {
		return period - shifted - complement;
	}
	else {
		return period + shifted;
	}
}

void checkMutedPulse(PulseChannel *pulse) {
	//if period is 8 or less, mute the channel
	if (pulse->wave.period <= 8 * TIMER_DOUBLE_SCALER) {
		pulse->wave.muted = 1;
	}
	//if sweeping will cause pulse period too high, also mute
	if (pulse->sweep_enable) {
		uint16_t target = doSweep(pulse->wave.period, pulse->sweep_shift, pulse->sweep_negate, pulse->sweep_complement);
		if (target > 0x7FF) pulse->wave.muted = 1;
	}
	//otherwise fine
	pulse->wave.muted = 0;
}

void clockPulse(PulseChannel *pulse, uint8_t step) {
	//envelope
	if (CLOCK_LOOKUP[frameCounter.mode][0][step]) {
		if (!pulse->const_volume_flag) {
			pulse->wave.envelope_counter = (pulse->wave.envelope_counter + 1) % pulse->volume; //also envelope reload
			if (pulse->wave.envelope_counter == 0) {
				if (pulse->wave.volume > 0) {
					--pulse->wave.volume;
				}
				else if (pulse->wave.volume == 0 && pulse->length_counter_halt) { //also loop flag
					pulse->wave.volume = 15;
				}
			}
		}
	}
	
	//length counter && sweep unit
	if (CLOCK_LOOKUP[frameCounter.mode][1][step]) {
		//length counter: decrement if length counter is not 0 nor halted
		if (pulse->wave.length_counter && !pulse->length_counter_halt) {
			--pulse->wave.length_counter;
		}
		//sweep
		if (pulse->sweep_enable) {
			//safety check
			uint8_t sweep_period = pulse->sweep_period == 0 ? 1 : pulse->sweep_period;
			pulse->wave.sweep_counter = (pulse->wave.sweep_counter + 1) % sweep_period;
			if (pulse->wave.sweep_counter == 0) {
				pulse->wave.period = doSweep(pulse->wave.period, pulse->sweep_shift, pulse->sweep_negate, pulse->sweep_complement);
				checkMutedPulse(pulse);
			}
		}
	}
}

void readPulse(uint8_t* buffer, const uint16_t buffer_len, PulseChannel* pulse) {
	if (pulse->wave.period == 0 || pulse->channel_muted) {
		//channel not fully initialized
		return;
	}
	
	//UPDATE AMPLITUDE BEFORE FEEDING TO BUFFER
	if (!(pulse->wave.enable && pulse->wave.length_counter && !pulse->wave.muted))
		return;
	pulse->wave.amplitude = (uint8_t)(pulse->wave.volume * PULSE_MIX_SCALE);
	
	#ifdef OPTIMIZE_SPEED
	for (uint16_t i = 0; i < buffer_len;) {
		int count = (pulse->wave.period - pulse->wave.counter + TIMER_HALF_PRESCALER-1) / TIMER_HALF_PRESCALER;
		if (count > (buffer_len-i)) {
			count = buffer_len-i;
		}
		memset(&buffer[i], PULSE_LOOKUP[pulse->duty][pulse->wave.pulse_counter] ? pulse->wave.amplitude : 0, count);
		i+= count;
		pulse->wave.counter = pulse->wave.counter + count*TIMER_HALF_PRESCALER;
		if (pulse->wave.counter >= pulse->wave.period) {
			pulse->wave.counter -= pulse->wave.period;
			pulse->wave.pulse_counter = (pulse->wave.pulse_counter + 1) % 8;
		}
	}
	#else
	#pragma unroll 16
	for (uint16_t i = 0; i < buffer_len; ++i) {
		if (PULSE_LOOKUP[pulse->duty][pulse->wave.pulse_counter]) {
			//REMEMBER TO RESET BUFFER TO 0
			buffer[i] += pulse->wave.amplitude;
		}
		
		//every 2*[period] CPU cycles, [pulse_counter] goes up by 1
		pulse->wave.counter = (pulse->wave.counter + TIMER_HALF_PRESCALER) % pulse->wave.period;
		if (pulse->wave.counter < TIMER_HALF_PRESCALER) {
			pulse->wave.pulse_counter = (pulse->wave.pulse_counter + 1) % 8;
		}
	}
	#endif
}


//==========================================//
//						TRIANGLE CHANNEL							//
//==========================================//
void updateTriangle(TriangleChannel *tri, uint8_t reg_num) {
	//length counter update only if 4th register is updated
	if (reg_num == 4 - 1) {
		if (tri->wave.enable)
			tri->wave.length_counter = LENGTH_COUNTER_LOOKUP[tri->length_counter];
		else 
			tri->wave.length_counter = 0;
	}
	
	//linear counter update only if 4th register is updated
	if (reg_num == 4 - 1) {
		tri->wave.linear_counter_flag = 1;
	}
	
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
uint8_t TRIANGLE_LOOKUP[32] = { 0 };
void initTriangleLookup(void) {
	for (uint8_t i = 0; i < 32; ++i) {
		//goes from 15 to 0 then 0 to 15
		TRIANGLE_LOOKUP[i] = (i < 16) ? (15 - i) : (i - 16);
		TRIANGLE_LOOKUP[i] = (uint8_t)(TRIANGLE_LOOKUP[i] * TRIANGLE_MIX_SCALE);
	}
}

void clockTriangle(TriangleChannel *tri, uint8_t step) {
	//linear counter
	if (CLOCK_LOOKUP[frameCounter.mode][0][step]) {
		if (tri->wave.linear_counter_flag) tri->wave.linear_counter = tri->counter_reload;
		else if (tri->wave.linear_counter > 0) --tri->wave.linear_counter;
		
		if (!tri->counter_flag) tri->wave.linear_counter_flag = 0;
	}
	
	//length counter
	if (CLOCK_LOOKUP[frameCounter.mode][1][step]) {
		//length counter: decrement if length counter is not 0 nor halted (counter flag = length counter halt)
		if (tri->wave.length_counter && !tri->counter_flag) {
			--tri->wave.length_counter;
		}
	}
}

void readTriangle(uint8_t* buffer, const uint16_t buffer_len, TriangleChannel* tri) {
	if (tri->wave.period == 0 || tri->wave.period == 1 || tri->channel_muted) {
		//channel not fully initialized
		return;
	}
	
	//UPDATE AMPLITUDE BEFORE FEEDING TO BUFFER
	//
	if (!(tri->wave.enable && tri->wave.length_counter && tri->wave.linear_counter))
		return;
	
	#ifdef OPTIMIZE_SPEED
	for (uint16_t i = 0; i < buffer_len;) {
		int count = (tri->wave.period - tri->wave.counter + TIMER_PRESCALER-1)/TIMER_PRESCALER;
		if (count > (buffer_len-i)) {
			count = buffer_len-i;
		}
		memset(&buffer[i], TRIANGLE_LOOKUP[tri->wave.amplitude_counter], count);
		i+= count;
		tri->wave.counter = tri->wave.counter + count*TIMER_PRESCALER;
		if (tri->wave.counter >= tri->wave.period) {
			tri->wave.counter -= tri->wave.period;
			tri->wave.amplitude_counter = (tri->wave.amplitude_counter + 1) % 32;
		}
	}
	#else
	#pragma unroll 16
	for (uint16_t i = 0; i < buffer_len; ++i) {
		//REMEMBER TO RESET BUFFER TO 0
		buffer[i] += TRIANGLE_LOOKUP[tri->wave.amplitude_counter];
		
		//every [period] CPU cycles, [amplitude_counter] goes up by 1
		tri->wave.counter = (tri->wave.counter + TIMER_PRESCALER) % tri->wave.period;
		if (tri->wave.counter < TIMER_PRESCALER) {
			tri->wave.amplitude_counter = (tri->wave.amplitude_counter + 1) % 32;
		}
	}
	#endif
}

void setTriangleTimer(TriangleChannel *tri, uint16_t timer) {
	tri->timer_lo = (uint8_t)(timer & 0xFF);
	tri->timer_hi = (uint8_t)((timer >> 8) & 0x07);
}


//==========================================//
//							NOISE CHANNEL								//
//==========================================//
const uint16_t NTSC_NOISE_PERIOD[] = {
	4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};
void updateNoise(NoiseChannel *noisy, uint8_t reg_num) {
	//length counter update only if 4th register is updated
	if (reg_num == 4 - 1) {
		if (noisy->wave.enable)
			noisy->wave.length_counter = LENGTH_COUNTER_LOOKUP[noisy->length_counter];
		else 
			noisy->wave.length_counter = 0;
	}
	
	//if constant volume, set default volume as input volume
	if (noisy->const_volume_flag) noisy->wave.volume = noisy->volume;
	//else, set volume to 15
	else if (reg_num == 4-1) noisy->wave.volume = 15;
	
	//get period from lookup table
	noisy->wave.period = TIMER_DOUBLE_SCALER * NTSC_NOISE_PERIOD[noisy->period_index];
	
	//counter check
	if (noisy->wave.counter >= noisy->wave.period) noisy->wave.counter = noisy->wave.counter % noisy->wave.period;
}

void clockNoise(NoiseChannel *noisy, uint8_t step) {
	//envelope
	if (CLOCK_LOOKUP[frameCounter.mode][0][step]) {
		if (!noisy->const_volume_flag) {
			noisy->wave.envelope_counter = (noisy->wave.envelope_counter + 1) % noisy->volume; //also envelope reload
			if (noisy->wave.envelope_counter == 0) {
				if (noisy->wave.volume > 0) {
					--noisy->wave.volume;
				}
				else if (noisy->wave.volume == 0 && noisy->length_counter_halt) { //also loop flag
					noisy->wave.volume = 15;
				}
			}
		}
	}
	
	//length counter
	if (CLOCK_LOOKUP[frameCounter.mode][1][step]) {
		//length counter: decrement if length counter is not 0 nor halted
		if (noisy->wave.length_counter && !noisy->length_counter_halt) {
			--noisy->wave.length_counter;
		}
	}
}

#define GET_BIT(X, BIT_POS) (((X) >> BIT_POS) & 0x01)   
void readNoise(uint8_t* buffer, const uint16_t buffer_len, NoiseChannel* noisy) {
	if (noisy->wave.period == 0 || noisy->channel_muted) {
		//channel not fully initialized
		return;
	}
	
	//UPDATE AMPLITUDE BEFORE FEEDING TO BUFFER
	if (noisy->wave.enable && noisy->wave.length_counter)
		noisy->wave.amplitude = (uint8_t)(noisy->volume * NOISE_MIX_SCALE);
	else
		noisy->wave.amplitude = 0;
	
	#pragma unroll 16
	for (uint16_t i = 0; i < buffer_len; ++i) {
		if (GET_BIT(noisy->shift_register, 0) == 1) {
			//REMEMBER TO RESET BUFFER TO 0
			buffer[i] += noisy->wave.amplitude;
		}
		
		//every 2*[period] CPU cycles, [shift_register] is updated
		noisy->wave.counter = (noisy->wave.counter + TIMER_HALF_PRESCALER) % noisy->wave.period;
		if (noisy->wave.counter < TIMER_HALF_PRESCALER) {
			uint8_t feedback = GET_BIT(noisy->shift_register, 0) ^ GET_BIT(noisy->shift_register, (noisy->mode ? 6 : 1));
			noisy->shift_register >>= 1;
			noisy->shift_register &= 16383; //set bit 14 to 0
			noisy->shift_register |= feedback << 14; //OR bit 14 with feedback
		}
	}
}
