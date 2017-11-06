#include "main.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

#include "machine/machine.h"
#include "mymath.h"

#include "control/parameters.h"

#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

using namespace Vert;

UART_HandleTypeDef huart1;

int main(void) __attribute__((optimize("O0")));

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

void test_IMUs();
void test_encoders();

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

Machine machine;

void playBootSound(){
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote('e', 7, 100);
	buzzer.addNote(0, 50);
	buzzer.addNote('d', 7, 100);
	buzzer.addNote(0, 50);
	buzzer.addNote('c', 7, 100);
	buzzer.addNote(0, 200);
	buzzer.addNote('g', 6, 150);
	buzzer.addNote(0, 150);
	buzzer.addNote('g', 7, 150);
	buzzer.addNote(0, 150);
	buzzer.addNote('e', 7, 100);
	buzzer.play();
}
void playStartSound2(){
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote('c', 5, 120);
	buzzer.addNote('g', 5, 120);
	buzzer.addNote('c', 6, 120);
	buzzer.addNote('d', 6, 120);
	buzzer.addNote('g', 6, 120);
	buzzer.addNote('c', 7, 120);
	buzzer.addNote('d', 7, 120);
	buzzer.addNote('c', 7, 120);
	buzzer.addNote('g', 6, 120);
	buzzer.addNote('d', 6, 120);
	buzzer.addNote('c', 6, 120);
	buzzer.addNote('g', 5, 120);
	buzzer.addNote('g', 5, 50);
	buzzer.addNote('c', 6, 50);
	buzzer.addNote('e', 6, 400);
	buzzer.play();
}
void playStartSound(){
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote('c', 7, 20);
	buzzer.addNote('e', 7, 20);
	buzzer.addNote('f', 7, 160);
	buzzer.addNote('e', 7, 200);
	buzzer.addNote('c', 7, 200);
	buzzer.addNote('g', 6, 200);
	buzzer.addNote('g', 6, 20);
	buzzer.addNote('c', 7, 20);
	buzzer.addNote('d', 7, 180);
	buzzer.addNote('c', 7, 240);
	buzzer.addNote(7459/4, 280);
	buzzer.addNote('f', 6, 320);
	buzzer.addNote(7459/4, 40);
	buzzer.addNote(4978/2, 40);
	buzzer.addNote('g', 7, 320);
	buzzer.play();
}
void playErrorSound(){
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote(2093, 130);
	buzzer.addNote(0, 20);
	buzzer.addNote(2093, 130);
	buzzer.addNote(0, 20);
	buzzer.addNote(2093, 130);
	buzzer.addNote(0, 20);
	buzzer.addNote(2093, 130);
	buzzer.addNote(0, 20);
	buzzer.play();
}
void playConfirmSound(){
	buzzer.stop();
	buzzer.addNote(0,1);
	buzzer.addNote(2093, 80);
	buzzer.addNote(1568, 80);
	buzzer.play();
}
void playConfirmSoundUp(){
	buzzer.stop();
	buzzer.addNote(0,1);
	buzzer.addNote(1568, 80);
	buzzer.addNote(2093, 80);
	buzzer.play();
}
void playEndSound(){
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote('b', 5, 250);
	buzzer.addNote(0, 300);
	buzzer.addNote('c', 6, 250);
	buzzer.addNote(0, 300);
	buzzer.addNote('c', 7, 150);
	buzzer.play();
}

uint8_t select(uint8_t max){
	int8_t val=0;
	int8_t changed=0;
	encoderR.reset();
	while(1){
		int16_t enc=encoderR.getCounter();
		if(enc<-500){
			val++; encoderR.reset(); changed=1;
		} else if(enc>500){
			val--; encoderR.reset(); changed=-1;
		}

		if(val<0) val=max; else if(val>max) val=0;

		if(changed!=0){
			/*
			// korikori
			motors.setOutput(0,changed*20);
			HAL_Delay(20);
			motors.setOutput(0, 0);
			*/
			if(max<8){
				for(uint8_t i=0;i<6;i+=2){ leds.write(i,val&(1<<(i/2))); leds.write(i+1,val&(1<<(i/2))); }
			} else {
				for(uint8_t i=0;i<6;i++){ leds.write(i,val&(1<<i)); }
			}
		}

		button.updateState();
		if(button.getState()==Button::RisingEdge){
			for(uint8_t i=0;i<6;i++) { leds.reset(i); }
			playConfirmSoundUp();
			return val;
		}

		HAL_Delay(20); changed=0;
	}
}

int8_t waitIR(){
	machine.unblock();
	for (uint8_t i = 0; i < 6; ++i) { leds.reset(i); }
	HAL_Delay(200);
	uint8_t cnt = 0;
	while(machine.getADCValue(IRSensor::RB)<2000){
		cnt++;
		button.updateState();
		if(button.getState()==Button::RisingEdge){
			playConfirmSound();
			return -1;
		}
		HAL_Delay(20);
		if(cnt%8==0){ for (uint8_t i = 0; i < 6; ++i) { leds.toggle(i); } }
	}
	for (uint8_t i = 0; i < 6; ++i) { leds.reset(i); }
	HAL_Delay(20);
	machine.block();
	playStartSound();
	return 0;
}

void initialiseRun(){
	//agent.setIndex(15);
	//agent.setDir(Maze::DirNorth);
	//agent.reroute();
	//index=15;
	//angle=0;
	//dir=Maze::DirNorth;

	//turnCount = 0;

	machine.deactivate();
	machine.block();
	imu1.init();
	imu2.init();
	machine.refreshIMUOffsets();
	machine.unblock();

	encoderL.reset();
	encoderR.reset();
	machine.setState(0,MyMath::Machine::InitialY,0);
	machine.resetTargetSequence(Trajectory::Position(0,MyMath::Machine::InitialY,0));
	machine.resetControllers();
	//machine.setWallAdjust();
	machine.setNeutralSideSensorValue();
	machine.activate();
	//isEndOfSequence = false;
}

int8_t trajMode(){
	using namespace MyMath;
	using namespace MyMath::Machine;
	using namespace Trajectory;
	//machine.setWallCorrection(false);

	uint8_t param=select(7);
	if(param==0) return 0;

	p_faststraight_start = pa_faststraight_start[param-1];
	p_faststraight = pa_faststraight[param-1];
	p_faststraight_end = pa_faststraight_end[param-1];
	p_fastturn = pa_fastturn[param-1];

	if(waitIR()<0) return 0;
	HAL_Delay(4000);

	initialiseRun();
	//machine.setWallCorrection(true);

	machine.pushTarget(Position(0,CellWidth/2,0), new MotionLinear(new EasingTrap()), p_straight_start);
	machine.pushTarget(Position(CellWidth/2,CellWidth,-MyMath::PI/2), new MotionSmoothArc(new EasingLinear()), p_turn);
	machine.pushTarget(Position(CellWidth,CellWidth,-MyMath::PI/2), new MotionLinear(new EasingTrap()), p_straight_end);

	while(1){
		if(!machine.isActivated()){
			playErrorSound();
			//flushLog();
			return -1;
		}
		if(machine.isTargetSequenceEmpty()){
			playErrorSound();
			//flushLog();
			break;
		}
		//flushLog();
		HAL_Delay(20);
	}
	HAL_Delay(3000);
	playConfirmSound();
	machine.deactivate();
	//flushLog();
	return 0;
}

void runMode(){
	uint8_t mode=select(6); int8_t result=0;
	switch(mode){
		case 0:  return;
		case 1:  return;//result=searchRunMode(); break;
		case 2:  return;//result=fastRunMode(false, false); break;
		case 3:  return;//result=fastRunMode(false, true); break;
		case 4:  return;//result=fastRunMode(true, true); break;
		case 5:  return;//result=fastRunMode(true, false); break;
		case 6:  result=trajMode(); break;
		default: break;
	}
	/*
	switch(result){
		case -1: buzzer; break;
		default: break;
	}
	*/
	return;
}

void menu(){
	uint8_t mode=select(5);
	switch(mode){
		case 0:  return;
		case 1:  return runMode();
		case 2:  return;// mazeMode();
		case 3:  return;// sensorMode();
		case 4:  return;//circuitMode(); return;
		case 5:  return;//goalSetMode(); return;
		default: break;
	}
	return;
}

void wait(){
	while(1){
		machine.block();
		button.updateState();
		if(button.getState()==Button::RisingEdge){
			// TODO: implement menu
			playConfirmSound(); menu();
		}
		HAL_Delay(20);
	}
}

int main(void)
{
	HAL_Init();

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

	MX_GPIO_Init();

	// NVIC (at least) should be configured after HAL_Init
	irsensor.configIRQ();
	machine.configIRQ();
	irled1.configIRQ();
	irled2.configIRQ();

	MX_USART1_UART_Init();

	__HAL_DBGMCU_FREEZE_TIM1();
	__HAL_DBGMCU_FREEZE_TIM2();

	encoderL.reset(); encoderR.reset();

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

	uint16_t buf[6];
	uint32_t batt_sum = 0;
	for(uint8_t i=0; i<200; i++){
		irsensor.startConv();
		HAL_Delay(1);
		while(!irsensor.isCompleted()){}
		irsensor.stopConv();
		irsensor.resetCompleted();
		irsensor.readValues(buf);
		batt_sum += buf[4];
	}
	if(batt_sum/200 <= 2250){
		playErrorSound();
		while(1);
	}

	playBootSound();

	//if(HAL_UART_Transmit(&huart1, "Bonjour\n", 8, 100) != HAL_OK){ Error_Handler(); }
	machine.unblock();

	float f = MyMath::sqrt(3.0f);
	char chrbuf[20]; uint8_t len = sprintf(chrbuf, "%f\n", f);
	if(HAL_UART_Transmit(&huart1, chrbuf, len, 100) != HAL_OK){ Error_Handler(); }

	/*
	select(7);

	waitIR();

	//while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)==GPIO_PIN_SET);

	HAL_Delay(2000);

	machine.refreshIMUOffsets();

	HAL_Delay(1000);
	machine.unblock();
	machine.activate();

	uint16_t cnt = 0;
	uint8_t lowVoltCnt = 0;

	//machine.startLogging();
	*/

	wait();

	/*
	float rw = 0;
	int8_t sig = 1;
	while (1)
	{
		uint32_t tic = HAL_GetTick();

		//test_IMUs();
		test_encoders();

		using namespace MyMath::Machine;

		if(cnt>=1000){ motors.setOutput(0,0); break; }

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

	playEndSound();

	machine.temp_setStatev(0);

	irsensor.stopConv();

	for (uint8_t i = 0; i < 6; ++i) {
		leds.reset(i);
	}
	cnt = 0;
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)==GPIO_PIN_SET){
		cnt++;
		if(cnt%64==0){
			for (uint8_t i = 0; i < 6; ++i) {
				leds.toggle(i);
			}
		}
		leds.update();
		HAL_Delay(1);
	}
	machine.stopLogging();

	machine.deactivate();

	volatile float* log = machine.log;
	uint16_t logcnt = 6000;
	for (uint16_t i = 0; i < logcnt; i+=4) {
		char str[100];
		uint8_t chrcnt = sprintf(str, "%4f %4f %4f %4f\n", log[i], log[i+1], log[i+2], log[i+3]);
		if(HAL_UART_Transmit(&huart1, str, chrcnt, 100) != HAL_OK){
			Error_Handler();
		}
	}

	while(1){
		cnt++;
		if(cnt%128==0){
			for (uint8_t i = 0; i < 6; ++i) {
				leds.toggle(i);
			}
		}
		leds.update();
		HAL_Delay(1);
	}
	*/
}

void test_IMUs(){
	// Testing IMUs
	volatile int16_t x = imu1.readInt16(59);
	volatile int16_t x2 = imu2.readInt16(59);
	if(x<0){ leds.reset(0); leds.set(1); }
	else { leds.reset(1); leds.set(0); }
	if(x2<0){ leds.reset(2); leds.set(3); }
	else { leds.reset(3); leds.set(2); }
}

void test_encoders(){
	// Testing encoders
	encoderL.captureSpeed();
	if(encoderL.speed()<0){ leds.reset(0); leds.set(1); }
	else if(encoderL.speed()==0) {  }
	else { leds.reset(1); leds.set(0); }
	encoderR.captureSpeed();
	if(encoderR.speed()<0){ leds.reset(2); leds.set(3); }
	else if(encoderR.speed()==0) {  }
	else { leds.reset(3); leds.set(2); }
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
