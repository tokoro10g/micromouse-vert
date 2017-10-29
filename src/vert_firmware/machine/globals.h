#pragma once

#include "encoder.h"
#include "buzzer.h"
#include "leds.h"
#include "motors.h"
#include "imu.h"
#include "irsensor.h"
#include "irled.h"
#include "machine.h"

extern Vert::Encoder encL;
extern Vert::Encoder encR;

extern Vert::Buzzer buzzer;
extern Vert::LEDs leds;
extern Vert::Motors motors;
extern Vert::IMU imu1;
extern Vert::IMU imu2;

extern Vert::IRSensor irsensor;
extern Vert::IRLED irled1;
extern Vert::IRLED irled2;
