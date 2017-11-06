#pragma once

#include "stm32f4xx_hal.h"

namespace Vert{
	class Button
	{
		public:
			Button(): internalState_(High) {
				GPIO_InitTypeDef GPIO_InitStruct;
				__HAL_RCC_GPIOA_CLK_ENABLE();
				GPIO_InitStruct.Pin = GPIO_PIN_12;
				GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
				GPIO_InitStruct.Pull = GPIO_PULLUP;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			}
			typedef enum {
				High,
				Low,
				RisingEdge,
				FallingEdge
			} ButtonState;

			void updateState(){
				bool state = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)==GPIO_PIN_SET);
				if(internalState_ == FallingEdge || internalState_ == RisingEdge){
					internalState_ = state ? High : Low;
				} else if(internalState_ == High && !state){
					internalState_ = FallingEdge;
				} else if(internalState_ == Low && state){
					internalState_ = RisingEdge;
				}
			}

			ButtonState getState() const{
				return internalState_;
			}
		private:
			ButtonState internalState_;
	};
}
