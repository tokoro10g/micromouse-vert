#pragma once
#include "stm32f4xx_hal.h"

namespace Vert {
	class IMU
	{
		public:
			IMU(SPI_TypeDef* SPI, GPIO_TypeDef* NSS_GPIO, uint16_t NSS_Pin):hspi_({}),gpio_(NSS_GPIO),pin_(NSS_Pin){
				GPIO_InitTypeDef GPIO_InitStruct;
				if(SPI==SPI1){
					__HAL_RCC_SPI1_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
					GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

					GPIO_InitStruct.Pin = GPIO_PIN_4;
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
				}
				if(SPI==SPI2){
					__HAL_RCC_SPI2_CLK_ENABLE();
					GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
					GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

					GPIO_InitStruct.Pin = GPIO_PIN_12;
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
					HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				}

				hspi_.Instance = SPI;
				hspi_.Init.Mode = SPI_MODE_MASTER;
				hspi_.Init.Direction = SPI_DIRECTION_2LINES;
				hspi_.Init.DataSize = SPI_DATASIZE_8BIT;
				hspi_.Init.CLKPolarity = SPI_POLARITY_LOW;
				hspi_.Init.CLKPhase = SPI_PHASE_1EDGE;
				hspi_.Init.NSS = SPI_NSS_SOFT;
				// 6.125MHz
				if(SPI == SPI1) hspi_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
				if(SPI == SPI2) hspi_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
				hspi_.Init.FirstBit = SPI_FIRSTBIT_MSB;
				hspi_.Init.TIMode = SPI_TIMODE_DISABLE;
				hspi_.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
				hspi_.Init.CRCPolynomial = 10;
				if (HAL_SPI_Init(&hspi_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
				HAL_SPI_MspInit(&hspi_);
			}

			void setNssState(bool state){
				if(state){
					gpio_->ODR |= pin_;
				} else {
					gpio_->ODR &= ~pin_;
				}
			}

			bool readReg(uint8_t reg, uint8_t &byte) {
				uint8_t data = reg | 0x80;
				uint8_t recv;
				setNssState(false);
				if(HAL_SPI_Transmit(&hspi_, &data, 1, 1) != HAL_OK) {
					setNssState(true);
					return false;
				}
				if(HAL_SPI_Receive(&hspi_, &recv, 1, 1) != HAL_OK) {
					setNssState(true);
					return false;
				}
				setNssState(true);
				byte = recv;
				return true;
			}
			bool writeReg(uint8_t reg, uint8_t val) {
				setNssState(false);
				if(HAL_SPI_Transmit(&hspi_, &reg, 1, 1) != HAL_OK) {
					setNssState(true);
					return false;
				}
				if(HAL_SPI_Transmit(&hspi_, &val, 1, 1) != HAL_OK) {
					setNssState(true);
					return false;
				}
				setNssState(true);
				return true;
			}

			volatile int16_t readInt16(uint8_t addr){
				union{
					uint16_t u;
					int16_t i;
				} _u2i;
				addr |= 0x80;
				uint8_t rx[2];
				setNssState(false);
				if(HAL_SPI_Transmit(&hspi_, &addr, 1, 1) != HAL_OK){
					setNssState(true);
					return -1;
				}
				if(HAL_SPI_Receive(&hspi_, rx, 2, 1) != HAL_OK){
					setNssState(true);
					return -1;
				}
				setNssState(true);
				_u2i.u=(rx[0]<<8)|rx[1];
				return _u2i.i;
			}

			bool test(){
				uint8_t reg;
				if(!readReg(117, reg)) return false;
				return reg==0x12;
			}

			bool reset(){
				if(!writeReg(107, 0x81)) return false;
				HAL_Delay(100);
				if(!test()) return false;
				return true;
			}

			bool init(){
				if(!reset()) return false;
				if(!writeReg(17, 0xc9)) return false;
				if(!writeReg(26, 0x00)) return false;
				if(!writeReg(27, 0x18)) return false;
				if(!writeReg(28, 0x18)) return false;
				if(!writeReg(29, 0x04)) return false;
				if(!writeReg(107, 0x01)) return false;
				return true;
			}

			void readAccXYZ(int16_t &x, int16_t &y, int16_t &z){
				union{
					uint16_t u;
					int16_t i;
				} _u2i;
				uint8_t addr = 59 | 0x80;
				unsigned char rx[6];
				setNssState(false);
				HAL_SPI_Transmit(&hspi_, &addr, 1, 1);
				HAL_SPI_Receive(&hspi_, rx, 6, 1);
				setNssState(true);
				_u2i.u=(rx[0]<<8)|rx[1];
				x = _u2i.i;
				_u2i.u=(rx[2]<<8)|rx[3];
				y = _u2i.i;
				_u2i.u=(rx[4]<<8)|rx[5];
				z = _u2i.i;
			}

			void readGyrXYZ(int16_t &x, int16_t &y, int16_t &z){
				union{
					uint16_t u;
					int16_t i;
				} _u2i;
				uint8_t addr = 67 | 0x80;
				unsigned char rx[6];
				setNssState(false);
				HAL_SPI_Transmit(&hspi_, &addr, 1, 1);
				HAL_SPI_Receive(&hspi_, rx, 6, 1);
				setNssState(true);
				_u2i.u=(rx[0]<<8)|rx[1];
				x = _u2i.i;
				_u2i.u=(rx[2]<<8)|rx[3];
				y = _u2i.i;
				_u2i.u=(rx[4]<<8)|rx[5];
				z = _u2i.i;
			}

		private:
			SPI_HandleTypeDef hspi_;
			GPIO_TypeDef* gpio_;
			uint16_t pin_;
	};
}
