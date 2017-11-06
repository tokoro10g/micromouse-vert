#pragma once

#include "stm32f4xx_hal.h"
#include "string.h"
#include "mymath.h"

namespace Vert {
	typedef struct {
		uint16_t freq;
		uint16_t duration;
	} Note;

	class Buzzer
	{
		public:
			Buzzer():htim_({}),noteBuffer_({}),isPlaying_(false),readPos_(0),writePos_(0){
				TIM_ClockConfigTypeDef sClockSourceConfig;
				TIM_MasterConfigTypeDef sMasterConfig;
				TIM_OC_InitTypeDef sConfigOC;
				GPIO_InitTypeDef GPIO_InitStruct;

				__HAL_RCC_TIM2_CLK_ENABLE();

				htim_.Instance = TIM2;
				htim_.Init.Prescaler = 999;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = 99;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				htim_.Init.RepetitionCounter = 0;
				if (HAL_TIM_Base_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				if (HAL_TIM_PWM_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
				sClockSourceConfig.ClockPolarity = 0;
				sClockSourceConfig.ClockFilter = 0;
				sClockSourceConfig.ClockPrescaler = 0;
				if (HAL_TIM_ConfigClockSource(&htim_, &sClockSourceConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(&htim_, &sMasterConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				__HAL_RCC_GPIOB_CLK_ENABLE();

				/**TIM2 GPIO Configuration    
				  PB10     ------> TIM2_CH3 
				  */
				GPIO_InitStruct.Pin = GPIO_PIN_10;
				GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

				sConfigOC.OCMode = TIM_OCMODE_PWM1;
				sConfigOC.Pulse = 0;
				sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
				sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
				sConfigOC.OCIdleState = 0;
				sConfigOC.OCNPolarity = 0;
				sConfigOC.OCNIdleState = 0;
				if (HAL_TIM_PWM_ConfigChannel(&htim_, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				if (HAL_TIM_PWM_Start(&htim_,TIM_CHANNEL_3) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
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

			void addNote(char note, uint8_t octave, uint16_t duration){
				addNote(makeFreq(note, octave), duration);
			}

			void addNote(uint16_t freq, uint16_t duration){
				Note note = {freq, duration};
				noteBuffer_[writePos_] = note;
				writePos_++;
				if(writePos_ >= noteCnt_) writePos_ = 0;
			}

			uint16_t makeFreq(char note, uint8_t octave){
				const uint16_t noteFrequencyBase[7] = {
					// oct=8
					//   C        C#       D        Eb       E        F       F#        G       G#        A       Bb        B
					//4186,    4435,    4699,    4978,    5274,    5588,    5920,    6272,    6645,    7040,    7459,    7902
					7040,    7902,  4186,    4699,    5274,    5588,    6272
				};

				if(octave > 8 || note > 'g' || note < 'a'){
					return 0;
				}
				return (uint16_t)((float)noteFrequencyBase[note-'a'] / (float)(1 << (8-octave)) +0.5f);
			}

		private:
			constexpr static uint8_t noteCnt_ = 255;
			constexpr static uint8_t pwm_ccr = 2;

			TIM_HandleTypeDef htim_;
			Note noteBuffer_[noteCnt_];
			bool isPlaying_;
			uint8_t readPos_;
			uint8_t writePos_;
	};
}
