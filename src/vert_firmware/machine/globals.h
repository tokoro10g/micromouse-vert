#pragma once

#include "button.h"
#include "buzzer.h"
#include "encoder.h"
#include "flash.h"
#include "imu.h"
#include "irled.h"
#include "irsensor.h"
#include "leds.h"
#include "motors.h"

#include "machine.h"

extern Vert::Button button;
extern Vert::Buzzer buzzer;
extern Vert::Encoder encoderL;
extern Vert::Encoder encoderR;
extern Vert::Flash flash;
extern Vert::IMU imu1;
extern Vert::IMU imu2;
extern Vert::IRLED irled1;
extern Vert::IRLED irled2;
extern Vert::IRSensor irsensor;
extern Vert::LEDs leds;
extern Vert::Motors motors;
