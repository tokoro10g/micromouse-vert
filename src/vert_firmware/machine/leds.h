#include "stm32f4xx_hal.h"

namespace Vert {
	constexpr static uint8_t L = 0; // Low
	constexpr static uint8_t H = 1; // High
	constexpr static uint8_t Z = 2; // High-Z
	// LED 4, 2, 6, 5, 3, 1; left-to-right, upper-to-lower
	const static uint8_t pattern_[6][3] = { {L,H,Z}, {H,L,Z}, {Z,H,L}, {Z,L,H}, {H,Z,L}, {L,Z,H} };
	class LEDs
	{
		public:
			LEDs():cursor_(0){
				GPIO_InitTypeDef GPIO_InitStruct;

				/* GPIO Port Clock Enable */
				__HAL_RCC_GPIOC_CLK_ENABLE();

				/*Configure GPIO pin Output Level */
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

				/*Configure GPIO pins : PC13 PC14 PC15 */
				GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
				HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
			}

			void set(uint8_t k){ output[k] = true; }
			void reset(uint8_t k){ output[k] = false; }
			void toggle(uint8_t k){ output[k] = !output[k]; }

			// called every 10ms (40 ticks)
			void update(){
				uint32_t moder = 0;
				moder |= (pattern_[cursor_][0] != 2) << (13 * 2);
				moder |= (pattern_[cursor_][1] != 2) << (14 * 2);
				moder |= (pattern_[cursor_][2] != 2) << (15 * 2);

				GPIOC->MODER = moder;

				uint32_t odr = 0;
				odr |= (pattern_[cursor_][0] == 1 && output[cursor_]) << 13;
				odr |= (pattern_[cursor_][1] == 1 && output[cursor_]) << 14;
				odr |= (pattern_[cursor_][2] == 1 && output[cursor_]) << 15;

				GPIOC->ODR = odr;

				cursor_++;
				if(cursor_ >= 6) cursor_ = 0;
			}

		private:
			uint8_t cursor_;
			bool output[6];
	};
}
