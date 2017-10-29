#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f4xx_hal.h"
#include "main.h"

void setIRSensorIRQObject(void* obj);
void setMachineIRQObject(void* obj);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TIM5_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
