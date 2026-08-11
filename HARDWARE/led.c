#include "led.h"


void led_show(uint8_t led,uint8_t state){
	//À¶É«
	if(led == 1){
		if(state == 1){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		}
	}
	//ÂÌÉ«
	else if(led == 2){
		if(state == 1){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		}
	}
	//ºìÉ«
	else{
		if(state == 1){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
		}
	}
	
}
