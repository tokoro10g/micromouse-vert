#include "stm32f4xx_hal.h"
#include "mymath.h"

namespace Vert {

	class Motors
	{
		public:
			Motors(){
				TIM_MasterConfigTypeDef sMasterConfig;
				TIM_OC_InitTypeDef sConfigOC;
				TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

				htim_.Instance = TIM1;
				htim_.Init.Prescaler = 1;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = ccr_max;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				htim_.Init.RepetitionCounter = 0;
				if (HAL_TIM_OC_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(&htim_, &sMasterConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
				sConfigOC.Pulse = 0;
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
				sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
				sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
				if (HAL_TIM_OC_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_OC_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_OC_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_OC_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
				sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
				sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
				sBreakDeadTimeConfig.DeadTime = 0;
				sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
				sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
				sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
				if (HAL_TIMEx_ConfigBreakDeadTime(&htim_, &sBreakDeadTimeConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				HAL_TIM_MspPostInit(&htim_);
			}

			void setOutput(int16_t left, int16_t right){
				MyMath::saturate(left, (int16_t)ccr_max);
				MyMath::saturate(right, (int16_t)ccr_max);
				if(left > 0){
					htim_.Instance->CCR1 = left;
					htim_.Instance->CCR2 = 0;
				} else {
					htim_.Instance->CCR1 = 0;
					htim_.Instance->CCR2 = -left;
				}
				if(right > 0){
					htim_.Instance->CCR3 = right;
					htim_.Instance->CCR4 = 0;
				} else {
					htim_.Instance->CCR3 = 0;
					htim_.Instance->CCR4 = -right;
				}
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
