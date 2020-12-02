#include "button.h"

ButtonStruct SW2 = {GPIOC, GPIO_PIN_15};
ButtonStruct SW3 = {GPIOB, GPIO_PIN_4};
ButtonStruct SW4 = {GPIOB, GPIO_PIN_5};
ButtonStruct SW5 = {GPIOB, GPIO_PIN_6};
ButtonStruct SW6 = {GPIOB, GPIO_PIN_7};
ButtonStruct SW7 = {GPIOB, GPIO_PIN_8};

//when a callback is defined in main, extern here and add to init
extern void SW5_pressed_callback(void);
extern void SW6_pressed_callback(void);
extern void SW4_pressed_callback(void);
extern void SW3_pressed_callback(void);
extern void SW7_pressed_callback(void);
extern void SW2_pressed_callback(void);
/**
	* Init button callbacks
	*/
void initButtons(void) {
	SW3.pressed_callback = SW3_pressed_callback;
	SW4.pressed_callback = SW4_pressed_callback;
	SW5.pressed_callback = SW5_pressed_callback;
	SW6.pressed_callback = SW6_pressed_callback;
	SW7.pressed_callback = SW7_pressed_callback;
	SW2.pressed_callback = SW2_pressed_callback;
}

/**
	* Update button states and call callbacks if needed
	* use the updateButtons() that (lazily) updates all buttons
	*/ 
void updateButton(Button button) {
	//while not pressed and ReadPin returns 0
	if (!HAL_GPIO_ReadPin(button->type, button->pin) && !button->pressed) {
		button->pressed = 1;
		//callback
		if (button->pressed_callback) button->pressed_callback();
	}
	//while not pressed and ReadPin returns 1
	if (HAL_GPIO_ReadPin(button->type, button->pin) && button->pressed) {
		button->pressed = 0;
		//callback
		if (button->released_callback) button->released_callback();
	}
}

void updateButtons(void) {
	updateButton(&SW2);
	updateButton(&SW3);
	updateButton(&SW4);
	updateButton(&SW5);
	updateButton(&SW6);
	updateButton(&SW7);
}
