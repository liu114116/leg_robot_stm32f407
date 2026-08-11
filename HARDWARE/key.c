#include "key.h"

uint8_t key_read(void){
	uint8_t state = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
	return state;
}
