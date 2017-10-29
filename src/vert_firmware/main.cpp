#include "main.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

#include "machine/encoder.h"
#include "machine/buzzer.h"
#include "machine/leds.h"
#include "machine/motors.h"
#include "machine/imu.h"
#include "machine/flash.h"

#include "machine/irsensor.h"

#ifdef __cplusplus
extern "C" {
#endif

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_TIM10_Init(void);
//static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);

void test_IMUs(Vert::IMU*, Vert::IMU*, Vert::LEDs*);
void test_encoders(Vert::Encoder*, Vert::Encoder*, Vert::LEDs*);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();

	Vert::Encoder encL(TIM3);
	Vert::Encoder encR(TIM4);

	Vert::Buzzer buzzer;
	Vert::LEDs leds;
	Vert::Motors motors;
	Vert::IMU imu1(SPI1, GPIOA, GPIO_PIN_4);
	Vert::IMU imu2(SPI2, GPIOB, GPIO_PIN_12);

	Vert::IRSensor irsensor;

	MX_USART1_UART_Init();

	//Vert::Flash log;
	//log.erase();

	__HAL_DBGMCU_FREEZE_TIM1();
	__HAL_DBGMCU_FREEZE_TIM2();

	encL.reset(); encR.reset();

	leds.reset(0);
	leds.reset(1);
	leds.reset(2);
	leds.reset(3);
	leds.reset(4);
	leds.reset(5);
	leds.update();

	HAL_Delay(500);

	if(!imu1.test()){ leds.reset(3); leds.reset(4); leds.reset(5); }
	if(!imu2.test()){ leds.reset(3); leds.reset(4); leds.reset(5); }

	if(!imu1.init()){ leds.reset(3); leds.reset(4); leds.reset(5); }
	if(!imu2.init()){ leds.reset(3); leds.reset(4); leds.reset(5); }

	buzzer.stop();
	buzzer.addNote(0, 200);
	buzzer.addNote(2093, 130);
	buzzer.addNote(0, 20);
	buzzer.addNote(1976, 130);
	buzzer.addNote(0, 20);
	buzzer.addNote(2093, 150);
	buzzer.addNote(0, 150);
	buzzer.addNote(1568, 150);
	buzzer.addNote(0, 150);
	buzzer.addNote(3136, 150);
	buzzer.addNote(0, 150);
	buzzer.addNote(2637, 150);
	buzzer.play();

	if(HAL_UART_Transmit(&huart1, "Bonjour\n", 8, 100) != HAL_OK){
		Error_Handler();
	}

	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)==GPIO_PIN_SET);

	HAL_Delay(2000);

	//motors.setOutput(-80,-80);
	motors.setOutput(0,0);

	HAL_TIM_OC_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim11, TIM_CHANNEL_1);

	irsensor.startConv();

	uint8_t cnt = 0;
	uint8_t lowVoltCnt = 0;
	volatile uint16_t* buf = irsensor.getBuffer();

	while(!irsensor.isCompleted()){}
	while (1)
	{
		uint32_t tic = HAL_GetTick();

		leds.update();
		buzzer.update();

		//test_IMUs(&imu1, &imu2, &leds);
		//test_encoders(&encL, &encR, &leds);

		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)==GPIO_PIN_RESET){
			motors.setOutput(150,150);
		} else {
			motors.setOutput(0, 0);
		}

		uint16_t irul = ((uint32_t)buf[3]+(uint32_t)buf[9]+(uint32_t)buf[15]+(uint32_t)buf[21]+(uint32_t)buf[27]+(uint32_t)buf[33]+(uint32_t)buf[39]+(uint32_t)buf[45])/8;
		uint16_t irur = ((uint32_t)buf[2]+(uint32_t)buf[8]+(uint32_t)buf[14]+(uint32_t)buf[20]+(uint32_t)buf[26]+(uint32_t)buf[32]+(uint32_t)buf[38]+(uint32_t)buf[44])/8;
		uint16_t irdl = ((uint32_t)buf[1]+(uint32_t)buf[7]+(uint32_t)buf[13]+(uint32_t)buf[19]+(uint32_t)buf[25]+(uint32_t)buf[31]+(uint32_t)buf[37]+(uint32_t)buf[43])/8;
		uint16_t irdr = ((uint32_t)buf[0]+(uint32_t)buf[6]+(uint32_t)buf[12]+(uint32_t)buf[18]+(uint32_t)buf[24]+(uint32_t)buf[30]+(uint32_t)buf[36]+(uint32_t)buf[42])/8;
		uint16_t irf = ((uint32_t)buf[5]+(uint32_t)buf[11]+(uint32_t)buf[17]+(uint32_t)buf[23]+(uint32_t)buf[29]+(uint32_t)buf[35]+(uint32_t)buf[41]+(uint32_t)buf[47])/8;

		if(irul>2000){ leds.set(0); }
		else { leds.reset(0); }
		if(irur>2000){ leds.set(1); }
		else { leds.reset(1); }
		if(irdl>800){ leds.set(2); }
		else { leds.reset(2); }
		if(irdr>550){ leds.set(3); }
		else { leds.reset(3); }
		if(irf>3100){ leds.set(4); leds.set(5); }
		else { leds.reset(4); leds.reset(5); }

		if(cnt%8==0){
			char str[7];
			sprintf(str, "%4d %4d %4d %4d %4d %4d\n", irul, irur, irdl, irdr, irf, buf[4]);
			if(HAL_UART_Transmit(&huart1, str, 30, 10000) != HAL_OK){
				Error_Handler();
			}
		}

		if(buf[4] <= 2150){
			lowVoltCnt++;
		} else {
			lowVoltCnt = 0;
		}
		if(lowVoltCnt > 200){
			break;
		}

		while(HAL_GetTick()-tic <= 1){}
		cnt++;
	}

	buzzer.stop();
	buzzer.update();

	motors.stop();

	irsensor.stopConv();

	TIM10->CCR1 = 0;
	TIM11->CCR1 = 0;
	HAL_TIM_OC_Stop(&htim10, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop(&htim11, TIM_CHANNEL_1);

	for (uint8_t i = 0; i < 6; ++i) {
		leds.reset(i);
	}
	cnt = 0;
	while(1){
		cnt++;
		if(cnt%64==0){
			for (uint8_t i = 0; i < 6; ++i) {
				leds.toggle(i);
			}
		}
		leds.update();
		HAL_Delay(1);
	}
}

void test_IMUs(Vert::IMU* imu1, Vert::IMU* imu2, Vert::LEDs* leds){
	// Testing IMUs
	volatile int16_t x = imu1->readInt16(59);
	volatile int16_t x2 = imu2->readInt16(59);
	if(x<0){ leds->reset(0); leds->set(1); }
	else { leds->reset(1); leds->set(0); }
	if(x2<0){ leds->reset(2); leds->set(3); }
	else { leds->reset(3); leds->set(2); }
}

void test_encoders(Vert::Encoder* encL, Vert::Encoder* encR, Vert::LEDs* leds){
	// Testing encoders
	encL->captureSpeed();
	if(encL->speed()<0){ leds->reset(0); leds->set(1); }
	else if(encL->speed()==0) {  }
	else { leds->reset(1); leds->set(0); }
	encR->captureSpeed();
	if(encR->speed()<0){ leds->reset(2); leds->set(3); }
	else if(encR->speed()==0) {  }
	else { leds->reset(3); leds->set(2); }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
		|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 49;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

  if (HAL_TIM_OC_Init(&htim10) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

  if (HAL_TIM_OnePulse_Init(&htim10, TIM_OPMODE_SINGLE) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

  HAL_TIM_MspPostInit(&htim10);

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 49;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 199;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

  if (HAL_TIM_OC_Init(&htim11) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

  if (HAL_TIM_OnePulse_Init(&htim11, TIM_OPMODE_SINGLE) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

  HAL_TIM_MspPostInit(&htim11);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 921600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pins : PH0 PH1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_IN_Button_Pin */
	GPIO_InitStruct.Pin = GPIO_IN_Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIO_IN_Button_GPIO_Port, &GPIO_InitStruct);

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if(htim->Instance==TIM10)
	{
		GPIO_InitStruct.Pin = IRLED1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
		HAL_GPIO_Init(IRLED1_GPIO_Port, &GPIO_InitStruct);
	}
	else if(htim->Instance==TIM11)
	{
		GPIO_InitStruct.Pin = IRLED2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
		HAL_GPIO_Init(IRLED2_GPIO_Port, &GPIO_InitStruct);
	}

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) { while(1){} }

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line){}

#endif

__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /*Configure the SysTick to have interrupt in 0.5ms time basis*/
  HAL_SYSTICK_Config(SystemCoreClock/2000);

  /*Configure the SysTick IRQ priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority ,0);

   /* Return function status */
  return HAL_OK;
}


#ifdef __cplusplus
}
#endif
