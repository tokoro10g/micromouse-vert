#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "machine/irsensor.h"
#include "machine/machine.h"

extern Vert::IRSensor irsensor;
extern Vert::IRLED irled1;
extern Vert::IRLED irled2;
extern Vert::Machine machine;

#ifdef __cplusplus
extern "C"{
#endif

static void EMG_ClearAllCCR(void) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

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
void  __attribute__((optimize("-O0"))) TIM5_IRQHandler(void) { machine.onTimerUpdate(); }

void  __attribute__((optimize("-O0"))) TIM1_UP_TIM10_IRQHandler(void) { irled1.onTimerUpdate(); }
void  __attribute__((optimize("-O0"))) TIM1_TRG_COM_TIM11_IRQHandler(void) { irled2.onTimerUpdate(); }

void  __attribute__((optimize("-O0"))) DMA2_Stream0_IRQHandler(void) { irsensor.onConvComplete(); }

#ifdef __cplusplus
}
#endif
