#pragma once
#include "stm32f4xx_hal.h"
#include "mymath.h"

namespace Vert {

	class Motors
	{
		public:
			Motors():htim_({}){
				TIM_MasterConfigTypeDef sMasterConfig;
				TIM_OC_InitTypeDef sConfigOC;
				TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
				GPIO_InitTypeDef GPIO_InitStruct;

				__HAL_RCC_TIM1_CLK_ENABLE();

				htim_.Instance = TIM1;
				htim_.Init.Prescaler = 1;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = 499;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				htim_.Init.RepetitionCounter = 0;
				HAL_TIM_PWM_MspInit(&htim_);
				if (HAL_TIM_PWM_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(&htim_, &sMasterConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				__HAL_RCC_GPIOA_CLK_ENABLE();

				/**TIM1 GPIO Configuration    
				  PA8     ------> TIM1_CH1
				  PA9     ------> TIM1_CH2
				  PA10     ------> TIM1_CH3
				  PA11     ------> TIM1_CH4 
				  */
				GPIO_InitStruct.Pin = MOTOR_RA_Pin|MOTOR_RB_Pin|MOTOR_LA_Pin|MOTOR_LB_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

				sConfigOC.OCMode = TIM_OCMODE_PWM1;
				sConfigOC.Pulse = 0;
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
				sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
				sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
				if (HAL_TIM_PWM_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_PWM_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_PWM_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_PWM_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
				sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
				sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
				sBreakDeadTimeConfig.DeadTime = 0;
				sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
				sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
				sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
				if (HAL_TIMEx_ConfigBreakDeadTime(&htim_, &sBreakDeadTimeConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				if (HAL_TIM_PWM_Start(&htim_,TIM_CHANNEL_1) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_PWM_Start(&htim_,TIM_CHANNEL_2) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_PWM_Start(&htim_,TIM_CHANNEL_3) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_PWM_Start(&htim_,TIM_CHANNEL_4) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
			}

			void setOutput(int16_t left, int16_t right){
				MyMath::saturate(left, (int16_t)ccr_max);
				MyMath::saturate(right, (int16_t)ccr_max);
				if(left > 0){
					htim_.Instance->CCR3 = left;
					htim_.Instance->CCR4 = 0;
				} else {
					htim_.Instance->CCR3 = 0;
					htim_.Instance->CCR4 = -left;
				}
				if(right > 0){
					htim_.Instance->CCR1 = right;
					htim_.Instance->CCR2 = 0;
				} else {
					htim_.Instance->CCR1 = 0;
					htim_.Instance->CCR2 = -right;
				}
			}

			void getOutput(int16_t& left, int16_t& right) const{
				left = htim_.Instance->CCR3 - htim_.Instance->CCR4;
				right = htim_.Instance->CCR1 - htim_.Instance->CCR2;
			}

			void stop(){
				htim_.Instance->CCR1 = 0;
				htim_.Instance->CCR2 = 0;
				htim_.Instance->CCR3 = 0;
				htim_.Instance->CCR4 = 0;
			}

		private:
			TIM_HandleTypeDef htim_;
			constexpr static uint16_t ccr_max = 499;
	};
}
