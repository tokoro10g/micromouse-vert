#pragma once
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include "globals.h"

namespace Vert{
	class Machine
	{
		public:
			Machine(): htim_{}, tickCounter(0) {
				TIM_ClockConfigTypeDef sClockSourceConfig;
				TIM_MasterConfigTypeDef sMasterConfig;

				__HAL_RCC_TIM5_CLK_ENABLE();
				HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
				HAL_NVIC_EnableIRQ(TIM5_IRQn);

				// 4kHz
				htim_.Instance = TIM5;
				htim_.Init.Prescaler = 24;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = 999;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				if (HAL_TIM_Base_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
				if (HAL_TIM_ConfigClockSource(&htim_, &sClockSourceConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(&htim_, &sMasterConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				setMachineIRQObject(this);
			}

			volatile void onTimerUpdate(){
				HAL_TIM_IRQHandler(&htim_);
				tickCounter++;

				switch(tickCounter%4){
					case 0:
						motorControl();
						// irled1.stopPulse();
						// irled2.stopPulse();
						// irsensor.startConv();
						break;
					case 1:
						motorControl();
						// while(!irsensor.isCompleted()){}
						// uint16_t sens[6];
						// irsensor.readValues(sens);
						// irled1.startPulse();
						// irled2.startPulse();
						// irsensor.startConv();
						break;
					case 2:
						motorControl();
						// while(!irsensor.isCompleted()){}
						// uint16_t sens[6];
						// irsensor.readValues(sens);
						break;
					case 3:
						poseControl();
						motorControl();
						break;
				}
			}

			void resetTick(){
				tickCounter = 0;
			}

			void readIRSensorUnlit(){
				// irled1.stopPulse();
				// irled2.stopPulse();
				// irsensor.startConv();
			}
			void readIRSensorLit(){
			}

			void motorControl(){
			}

			void poseControl(){
			}

		private:
			TIM_HandleTypeDef htim_;
			volatile uint32_t tickCounter;
	};
}
