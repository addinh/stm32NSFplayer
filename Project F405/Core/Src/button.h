#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f4xx_hal.h"

typedef struct {
	GPIO_TypeDef* type;
	uint16_t pin;
	
	uint8_t pressed;
	void (*pressed_callback)(void);
	void (*released_callback)(void);
} ButtonStruct, *Button;


void initButtons(void);
void updateButtons(void);

#endif
