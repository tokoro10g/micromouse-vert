#include "encoder.h"
#include "buzzer.h"
#include "leds.h"
#include "motors.h"
#include "imu.h"
#include "irsensor.h"
#include "irled.h"
#include "machine.h"

Vert::Encoder encL(TIM3);
Vert::Encoder encR(TIM4);

Vert::Buzzer buzzer;
Vert::LEDs leds;
Vert::Motors motors;
Vert::IMU imu1(SPI1, GPIOA, GPIO_PIN_4);
Vert::IMU imu2(SPI2, GPIOB, GPIO_PIN_12);

Vert::IRSensor irsensor;
Vert::IRLED irled1(TIM10);
Vert::IRLED irled2(TIM11);

Vert::Machine machine;
