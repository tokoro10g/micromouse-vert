#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "machine/irsensor.h"
#include "machine/machine.h"

void* g_irsensor=0;
void* g_machine=0;

#ifdef __cplusplus
extern "C"{
#endif

void setIRSensorIRQObject(void* obj){
	g_irsensor = obj;
}
void setMachineIRQObject(void* obj){
	g_machine = obj;
}

static void EMG_ClearAllCCR(void) {
  // Stop motors
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;

  // Stop IR LEDs
  TIM10->CCR1 = 0;
  TIM11->CCR1 = 0;

  // Stop buzzer
  TIM2->CCR3 = 0;
}

extern TIM_HandleTypeDef htim5;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void) { EMG_ClearAllCCR(); }

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void) { EMG_ClearAllCCR(); while(1){} }

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void) { EMG_ClearAllCCR(); while(1){} }

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void) { EMG_ClearAllCCR(); while(1){} }

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void) { EMG_ClearAllCCR(); while(1){} }

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void) { }

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void) { }

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void) { }

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void) { HAL_IncTick(); HAL_SYSTICK_IRQHandler(); }

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
*/
void TIM1_BRK_TIM9_IRQHandler(void) {
	//HAL_TIM_IRQHandler(&htim9);
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void) { static_cast<Vert::Machine*>(g_machine)->onTimerUpdate(); }

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void) { static_cast<Vert::IRSensor*>(g_irsensor)->onConvComplete(); }

#ifdef __cplusplus
}
#endif
