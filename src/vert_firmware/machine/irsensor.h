#pragma once

#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

namespace Vert{
	class IRSensor {
		public:
			IRSensor():hadc_{},hdma_{},buffer_{},isCompleted_(false){
				ADC_ChannelConfTypeDef sConfig;
				GPIO_InitTypeDef GPIO_InitStruct;

				__HAL_RCC_ADC1_CLK_ENABLE();

				hadc_.Instance = ADC1;
				hadc_.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
				hadc_.Init.Resolution = ADC_RESOLUTION_12B;
				hadc_.Init.ScanConvMode = ENABLE;
				hadc_.Init.ContinuousConvMode = DISABLE;
				hadc_.Init.DiscontinuousConvMode = ENABLE;
				hadc_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
				hadc_.Init.ExternalTrigConv = ADC_SOFTWARE_START;
				hadc_.Init.DataAlign = ADC_DATAALIGN_RIGHT;
				hadc_.Init.DMAContinuousRequests = DISABLE;
				hadc_.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
				hadc_.Init.NbrOfDiscConversion = 6;

				if (HAL_ADC_Init(&hadc_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				const uint32_t scanChannels[6][2] = { {ADC_CHANNEL_0, 1}, {ADC_CHANNEL_1, 2}, {ADC_CHANNEL_2, 3}, {ADC_CHANNEL_3, 4}, {ADC_CHANNEL_8, 5}, {ADC_CHANNEL_9, 6} };
				sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
				for (uint8_t i = 0; i < 6; ++i) {
					sConfig.Channel = scanChannels[i][0];
					sConfig.Rank = scanChannels[i][1];
					if (HAL_ADC_ConfigChannel(&hadc_, &sConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				}

				__HAL_RCC_GPIOA_CLK_ENABLE();
				__HAL_RCC_GPIOB_CLK_ENABLE();

				GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
				GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

				GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

				__HAL_RCC_DMA2_CLK_ENABLE();
				HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
				HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

				hdma_.Instance = DMA2_Stream0;
				hdma_.Init.Channel = DMA_CHANNEL_0;
				hdma_.Init.Direction = DMA_PERIPH_TO_MEMORY;
				hdma_.Init.PeriphInc = DMA_PINC_DISABLE;
				hdma_.Init.MemInc = DMA_MINC_ENABLE;
				hdma_.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
				hdma_.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
				hdma_.Init.Mode = DMA_CIRCULAR;
				hdma_.Init.Priority = DMA_PRIORITY_MEDIUM;
				hdma_.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
				if (HAL_DMA_Init(&hdma_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				__HAL_LINKDMA(&hadc_,DMA_Handle,hdma_);

				setIRSensorIRQObject(this);
			}

			void startConv(){
				HAL_ADC_Start_DMA(&hadc_, (uint32_t*)buffer_, 6);
			}

			void stopConv(){
				HAL_ADC_Stop_DMA(&hadc_);
			}

			void onConvComplete(){
				HAL_DMA_IRQHandler(&hdma_);
				isCompleted_ = true;
			}

			bool isCompleted() const{ return isCompleted_; }
			void resetCompleted() { isCompleted_ = false; }

			volatile uint16_t* getBuffer(){
				return buffer_;
			}

		private:
			ADC_HandleTypeDef hadc_;
			DMA_HandleTypeDef hdma_;
			volatile uint16_t buffer_[6];
			volatile bool isCompleted_;
	};
}
