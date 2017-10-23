#include "stm32f4xx_hal.h"

namespace Vert {
	typedef struct {
		uint16_t freq;
		uint16_t duration;
	} Note;

	class Buzzer
	{
		public:
			Buzzer():isPlaying_(false),readPos_(0),writePos_(0){
				TIM_ClockConfigTypeDef sClockSourceConfig;
				TIM_MasterConfigTypeDef sMasterConfig;
				TIM_OC_InitTypeDef sConfigOC;

				htim_.Instance = TIM2;
				htim_.Init.Prescaler = 999;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = 99;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				if (HAL_TIM_Base_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
				if (HAL_TIM_ConfigClockSource(&htim_, &sClockSourceConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				if (HAL_TIM_OC_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(&htim_, &sMasterConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
				sConfigOC.Pulse = 0;
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
				if (HAL_TIM_OC_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				HAL_TIM_MspPostInit(&htim_);
			}

			bool isPlaying() const{ return isPlaying_; }

			// called every 1ms (4 ticks) (
			void update(){
				if(!isPlaying_) return;
				if(noteBuffer_[readPos_].duration == 0){
					readPos_++;
					if(readPos_ >= noteCnt_) readPos_ = 0;
					if(noteBuffer_[readPos_].freq > 0 && noteBuffer_[readPos_].duration > 0){
						htim_.Instance->CCR3 = pwm_ccr;
						setFreq(noteBuffer_[readPos_].freq);
					} else {
						if(noteBuffer_[readPos_].duration > 0){
							// pause note
							htim_.Instance->CCR3 = 0;
						} else {
							stop();
						}
					}
				} else {
					noteBuffer_[readPos_].duration--;
				}
			}

			void play(){ isPlaying_ = true; }
			void stop(){ htim_.Instance->CCR3 = 0; isPlaying_ = false; writePos_ = 0; readPos_ = 0; }

			void setFreq(uint16_t freq){
				htim_.Instance->PSC = 1000000UL/freq - 1;
			}

			void addNote(uint16_t freq, uint16_t duration){
				Note note = {freq, duration};
				noteBuffer_[writePos_] = note;
				writePos_++;
				if(writePos_ >= noteCnt_) writePos_ = 0;
			}

		private:
			constexpr static uint8_t noteCnt_ = 128;
			constexpr static uint8_t pwm_ccr = 29;

			TIM_HandleTypeDef htim_;
			Note noteBuffer_[noteCnt_];
			bool isPlaying_;
			uint8_t readPos_;
			uint8_t writePos_;
	};
}
