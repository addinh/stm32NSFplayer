#include <stdio.h>
#include <stdint.h>
#include <integer.h>

#pragma anon_unions

typedef union{
	
	struct{
		char NES_sound_file[5];
		BYTE version;
		BYTE total_song;
		BYTE starting_song;
		BYTE load_address_orgin[2];
		BYTE init_address_orgin[2];
		BYTE play_address_orgin[2];
		char name[32];
		char artist[32];
		char copy_right[32];
		BYTE play_speed_NTSC[2];
		BYTE Bankswitch[8];
		BYTE play_speed_PAL[2];
		BYTE play_speed_indicator;
		BYTE chip_support;
		BYTE NSF2_reserved;
		BYTE program_data[3];
	}format;
	uint8_t text[128];
	
} nsf_file;

