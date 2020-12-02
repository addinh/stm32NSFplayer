#ifndef EMULATOR6502_H_
#define EMULATOR6502_H_

#include "stdint.h"

#pragma anon_unions

#define WORD uint16_t
#define BYTE uint8_t
#define UINT uint32_t

typedef union
{
	WORD						W;
	struct{ BYTE l; BYTE h; }	B;
} TWIN;

typedef union
{
	UINT								D;
	struct{ BYTE l; BYTE h; WORD w; }	B;
} QUAD;

typedef BYTE (*ReadProc)(WORD);
typedef void (*WriteProc)(WORD,BYTE);

extern BYTE		pRAM[0x0800];			//RAM:		0x0000 - 0x07FF
extern	BYTE		pSRAM[0x2000];			//SRAM:		0x6000 - 0x7FFF (non-FDS only)

//extern	BYTE		pAPU[0x0020];			//APU:		0x4000 - 0x4017

extern	BYTE		pExRAM[0x1000];			//ExRAM:	0x5C00 - 0x5FF5 (MMC5 only)
								// Also holds NSF player info (at 0x5000 - 0x500F)
								
#define MAX_ROM_SIZE 0x9000
extern	BYTE		pROM_Full[MAX_ROM_SIZE];		//Full ROM buffer

extern	BYTE*		pROM[10];		//ROM banks (point to areas in pROM_Full)
								//0x8000 - 0xFFFF
								//also includes 0x6000 - 0x7FFF (FDS only)
extern	BYTE*		pStack;			//the stack (points to areas in pRAM)
								//0x0100 - 0x01FF

//	int			nROMSize;		//size of this ROM file in bytes
//	int			nROMBankCount;	//max number of 4k banks
//	int			nROMMaxSize;	//size of allocated pROM_Full buffer

	/*
	 *	Memory Proc Pointers
	 */

extern	ReadProc	ReadMemory[0x10];
extern	WriteProc	WriteMemory[0x10];

	/*
	 *	6502 Registers / Mode
	 */

extern	BYTE		regA;			// Accumulator
extern	BYTE		regX;			// X-Index
extern	BYTE		regY;			// Y-Index
extern	BYTE		regP;			// Processor Status
extern	BYTE		regSP;			// Stack Pointer
extern	WORD		regPC;			// Program Counter

extern	BYTE		bPALMode;		// 1 if in PAL emulation mode, 0 if in NTSC
extern	BYTE		bCPUJammed;		// 0 = not jammed.  1 = really jammed.  2 = 'fake' jammed
								//  fake jam caused by the NSF code to signal the end of the play/init routine


//	BYTE		nMultIn_Low;	//Multiplication Register, for MMC5 chip only (5205+5206)
//	BYTE		nMultIn_High;


//	/*
//	 *	NSF Preparation Information
//	 */

extern BYTE		nBankswitchInitValues[10];	//banks to swap to on tune init
//	WORD		nPlayAddress;				// Play routine address
//	WORD		nInitAddress;				// Init routine address

//	BYTE		nExternalSound;				// external sound chips
//	BYTE		nCurTrack;

//	float		fNSFPlaybackSpeed;

//	/*
//	 *	pAPU
//	 */

//	BYTE		nFrameCounter;		//Frame Sequence Counter
//	BYTE		nFrameCounterMax;	//Frame Sequence Counter Size (3 or 4 depending on $4017.7)
//	BYTE		bFrameIRQEnabled;	//TRUE if frame IRQs are enabled
//	BYTE		bFrameIRQPending;	//TRUE if the frame sequencer is holding down an IRQ

//	BYTE		bChannelMix[24];	//Mixing flags for each channel (except the main 5!)
//	BYTE		nChannelVol[29];	//Volume settings for each channel
//	char		nChannelPan[29];	//Panning for each channel

//	/*
//	 *	Timing and Counters
//	 */
//	float		fTicksUntilNextFrame;	//clock cycles until the next frame

//	float		fTicksPerPlay;			//clock cycles between play calls
//	float		fTicksUntilNextPlay;	//clock cycles until the next play call

//	float		fTicksPerSample;		//clock cycles between generated samples
//	float		fTicksUntilNextSample;	//clocks until the next sample

extern	UINT		nCPUCycle;
extern	UINT		nAPUCycle;
//	UINT		nTotalPlays;			//number of times the play subroutine has been called (for tracking output time)


//	/*
//	 *	Silence Tracker
//	 */
//	int			nSilentSamples;
//	int			nSilentSampleMax;
//	int			nSilenceTrackMS;
//	BYTE		bNoSilenceIfTime;
//	BYTE		bTimeNotDefault;

//	//
//	//	Sound output options
//	//
//	int				nSampleRate;
//	int				nMonoStereo;

//	//
//	//	Volume/fading/filter tracking
//	//

//	float			fMasterVolume;

//	UINT			nStartFade;					//play call to start fading out
//	UINT			nEndFade;					//play call to stop fading out (song is over)
//	BYTE			bFade;						//are we fading?
//	float			fFadeVolume;
//	float			fFadeChange;

//	/*
//	 *	Designated Output Buffer
//	 */
//	BYTE*		pOutput;
//	int			nDownsample;

	/*
	 *	Misc Options
	 */
//	BYTE		bDMCPopReducer;					//1 = enabled, 0 = disabled
//	BYTE		nDMCPop_Prev;
//	BYTE		bDMCPop_Skip;
//	BYTE		bDMCPop_SamePlay;

//	BYTE		nForce4017Write;
//	BYTE		bN106PopReducer;
//	int			nInvertCutoffHz;
//	BYTE		bIgnore4011Writes;

extern	BYTE		bIgnoreBRK;
extern	BYTE		bIgnoreIllegalOps;
//extern	BYTE		bNoWaitForReturn;
//extern	BYTE		bPALPreference;
//extern	BYTE		bCleanAXY;
//extern	BYTE		bResetDuty;

//	/*
//	 *	Sound Filter
//	 */

//	__int64		nFilterAccL;
//	__int64		nFilterAccR;
//	__int64		nFilterAcc2L;
//	__int64		nFilterAcc2R;
//	__int64		nHighPass;
//	__int64		nLowPass;

//	int			nHighPassBase;
//	int			nLowPassBase;

//	BYTE		bHighPassEnabled;
//	BYTE		bLowPassEnabled;
//	BYTE		bPrePassEnabled;

//	int			nSmPrevL;
//	int			nSmAccL;
//	int			nSmPrevR;
//	int			nSmAccR;
//	int			nPrePassBase;
//	float		fSmDiv;


extern TWIN front;
extern TWIN final;
extern BYTE val;
extern BYTE op;

UINT emulate6502(UINT runto);



/**
 * Reimplementing the read/write here
 */
static inline BYTE 		ReadMemory_RAM(WORD a)				{ return pRAM[a & 0x07FF]; }
static inline BYTE 		ReadMemory_ExRAM(WORD a)			{ return pExRAM[a & 0x0FFF]; }
static inline BYTE 		ReadMemory_SRAM(WORD a)				{ return pSRAM[a & 0x1FFF]; }
inline BYTE 		ReadMemory_pAPU(WORD a);//				{ return pAPU[a & 0x001F]; } //TODO read from wave structs
static inline BYTE 		ReadMemory_ROM(WORD a)				{ return pROM[(a >> 12) - 6][a & 0x0FFF]; }
static inline BYTE 		ReadMemory_Default(WORD a)			{ return (a >> 8); }


static inline void 		WriteMemory_RAM(WORD a,BYTE v)		{ pRAM[a & 0x07FF] = v; }
static inline void 		WriteMemory_ExRAM(WORD a,BYTE v) {
	pExRAM[a & 0x0FFF] = v;
	
	//do bank switching
	if (a < 0x5FF6) return;
	uint8_t bank = a - 0x5FF6;
	if (bank >= 2) pROM[bank] = pROM_Full + (v << 12);
	else pROM[bank] = pSRAM + (v << 12);
}
static inline void 		WriteMemory_SRAM(WORD a,BYTE v)		{ pSRAM[a & 0x1FFF] = v; }
inline void 		WriteMemory_pAPU(WORD a,BYTE v);	//{ pAPU[a & 0x001F] = v; } //TODO write to wave structs
static inline void 		WriteMemory_Default(WORD a,BYTE v)	{ }


#endif /* EMULATOR6502_H_ */
