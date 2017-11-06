#pragma once
#include "stm32f4xx_hal.h"

namespace Vert{
	class IRLED {
		public:
			IRLED(TIM_TypeDef* TIM):htim_({}){
				GPIO_InitTypeDef GPIO_InitStruct;

				__HAL_RCC_GPIOB_CLK_ENABLE();

				if(TIM==TIM10){
					__HAL_RCC_TIM10_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_8;
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				} else if(TIM==TIM11){
					__HAL_RCC_TIM11_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_9;
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
				}

				htim_.Instance = TIM;
				htim_.Init.Prescaler = 49;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = 199;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				if (HAL_TIM_Base_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				HAL_TIM_Base_Start_IT(&htim_);
			}

			void configIRQ(){
				if(htim_.Instance == TIM10){
					HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
					HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
				} else if(htim_.Instance == TIM11){
					HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1, 0);
					HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
				}
			}

			void onTimerUpdate(){
				HAL_TIM_IRQHandler(&htim_);
				stopPulse();
			}

			void triggerPulse(){
				if(htim_.Instance == TIM10){
					TIM10->CNT = 0;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
				} else if(htim_.Instance == TIM11){
					TIM11->CNT = 0;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
				}
			}

			void stopPulse(){
				if(htim_.Instance == TIM10){
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
				} else if(htim_.Instance == TIM11){
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
				}
			}
		private:
			TIM_HandleTypeDef htim_;
	};
}
