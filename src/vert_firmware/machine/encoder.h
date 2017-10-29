#pragma once

#include "stm32f4xx_hal.h"
#include "../../mymath/mymath.h"

namespace Vert {
	class Encoder
	{
		public:
			Encoder(TIM_TypeDef* TIM):htim_({}),prevCnt_(0),speed_(0) {
				TIM_Encoder_InitTypeDef sConfig;
				TIM_MasterConfigTypeDef sMasterConfig;
				GPIO_InitTypeDef GPIO_InitStruct;

				__HAL_RCC_GPIOB_CLK_ENABLE();
				if(TIM==TIM3) {
					__HAL_RCC_TIM3_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				} else if(TIM==TIM4) {
					__HAL_RCC_TIM4_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
					GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				}

				htim_.Instance = TIM;
				htim_.Init.Prescaler = 0;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = 65535;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

				sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
				sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
				sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
				sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
				sConfig.IC1Filter = 0;
				sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
				sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
				sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
				sConfig.IC2Filter = 0;
				if (HAL_TIM_Encoder_Init(&htim_, &sConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(&htim_, &sMasterConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				HAL_TIM_Encoder_Start(&htim_, TIM_CHANNEL_ALL);
			}
			void reset() { prevCnt_ = 0; speed_ = 0; htim_.Instance->CNT = 0; }
			int16_t speed() const{ return speed_; }
			void captureSpeed() {
				uint16_t nowCnt = htim_.Instance->CNT;
				int32_t diff = nowCnt - prevCnt_;
				prevCnt_ = nowCnt;
				if(diff > 32767 || diff < -32767){
					speed_ = diff - (int32_t)MyMath::signof(diff) * 65536;
				} else {
					speed_ = diff;
				}
			}

		private:
			TIM_HandleTypeDef htim_;
			uint16_t prevCnt_;
			int16_t speed_;
	};
}
