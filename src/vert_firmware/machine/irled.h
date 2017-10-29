#pragma once
#include "stm32f4xx_hal.h"

namespace Vert{
	class IRLED {
		public:
			IRLED(TIM_TypeDef* TIM):htim_({}){
				TIM_OC_InitTypeDef sConfigOC;
				GPIO_InitTypeDef GPIO_InitStruct;

				__HAL_RCC_GPIOB_CLK_ENABLE();

				if(TIM==TIM10){
					__HAL_RCC_TIM10_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_8;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				} else if(TIM==TIM11){
					__HAL_RCC_TIM11_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_9;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				}

				htim_.Instance = TIM;
				htim_.Init.Prescaler = 49;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = 199;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				if (HAL_TIM_Base_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				if (HAL_TIM_OC_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				if (HAL_TIM_OnePulse_Init(&htim_, TIM_OPMODE_SINGLE) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
				sConfigOC.Pulse = 99;
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
				if (HAL_TIM_OC_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
			}

			void triggerPulse(){
				HAL_TIM_OnePulse_Start(&htim_, TIM_CHANNEL_1);
			}

			void stopPulse(){
				HAL_TIM_OnePulse_Stop(&htim_, TIM_CHANNEL_1);
			}
		private:
			TIM_HandleTypeDef htim_;
	};
}
