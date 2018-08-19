#pragma once

#include "stm32f4xx_hal.h"

namespace Vert {

	class Flash{
		public:
			Flash(){}

			bool erase(){
				FLASH_EraseInitTypeDef EraseInitStruct;
				EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
				EraseInitStruct.Sector = FLASH_SECTOR_7;
				EraseInitStruct.NbSectors = 1;
				EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
				__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_PGPERR|FLASH_FLAG_WRPERR);

				uint32_t PageError = 0;
				HAL_FLASH_Unlock();
				if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
					HAL_FLASH_Lock();
					return false;
				}
				HAL_FLASH_Lock();
				return true;
			}
			bool writeU32(const uint32_t addr, const uint32_t data){
				bool b = false;
				if(addr0 + addr > addrf) return false;
				HAL_FLASH_Unlock();
				if(HAL_FLASH_Program(TYPEPROGRAM_WORD, addr0 + addr, data) == HAL_OK) b = true;
				HAL_FLASH_Lock();
				return b;
			}
			bool readU32(const uint32_t addr, uint32_t& data){
				if(addr0 + addr > addrf) return false;
				HAL_FLASH_Unlock();
				data = *reinterpret_cast<volatile uint32_t*>(addr0 + addr);
				HAL_FLASH_Lock();
				return true;
			}
			bool writeU8(const uint32_t addr, const uint8_t data){
				bool b = false;
				if(addr0 + addr > addrf) return false;
				HAL_FLASH_Unlock();
				if(HAL_FLASH_Program(TYPEPROGRAM_BYTE, addr0 + addr, data) == HAL_OK) b = true;
				HAL_FLASH_Lock();
				return b;
			}
			bool readU8(const uint32_t addr, uint8_t& data){
				if(addr0 + addr > addrf) return false;
				HAL_FLASH_Unlock();
				data = *reinterpret_cast<volatile uint8_t*>(addr0 + addr);
				HAL_FLASH_Lock();
				return true;
			}
			bool writeF32(const uint32_t addr, const float data){
				bool b = false;
				if(addr0 + addr > addrf) return false;
				HAL_FLASH_Unlock();
				u2f_.f = data;
				if(HAL_FLASH_Program(TYPEPROGRAM_WORD, addr0 + addr, u2f_.u) == HAL_OK) b = true;
				HAL_FLASH_Lock();
				return b;
			}
			bool readF32(const uint32_t addr, float& data){
				if(addr0 + addr > addrf) return false;
				HAL_FLASH_Unlock();
				u2f_.u = *reinterpret_cast<volatile uint32_t*>(addr0 + addr);
				data = u2f_.f;
				HAL_FLASH_Lock();
				return true;
			}

		private:
			union{
					uint32_t u;
					float f;
				} u2f_;
			constexpr static uint32_t addr0 = 0x08060000;
			constexpr static uint32_t addrf = 0x0807FFFF;
	};
}
