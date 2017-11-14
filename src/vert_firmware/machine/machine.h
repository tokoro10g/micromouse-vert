#pragma once

#include <algorithm>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include "globals.h"
#include "control/pidcontroller.h"
#include "mymath.h"
#include "mymath_machine.h"

#include "../../libtrajectory/targetsequence.h"
#include "../../libtrajectory/target.h"
#include "../../libtrajectory/position.h"
#include "../../libtrajectory/functors.h"

namespace Vert{
	class Machine
	{
		private:
			constexpr static float dt = 250e-6f;

			TIM_HandleTypeDef htim_;
			volatile uint32_t tickCounter;
			volatile bool isActivated_;
			volatile bool isBlocked_;

			Vert::PIDController<float> motorLController;
			Vert::PIDController<float> motorRController;

			struct State {
				float x, y, phi, rx, ry, rphi, rvx, rvy, rw, v, w, wsens;
				State():x(0),y(MyMath::Machine::InitialY),phi(0),rx(0),ry(0),rphi(0),rvx(0),rvy(0),rw(0),v(0),w(0),wsens(0){}
				void update(int16_t vl, int16_t vr) volatile{
					phi+=wsens/2.f; phi = MyMath::normalized(phi, MyMath::Machine::PIInGyroValue);
					float phiInRadian=MyMath::Machine::convertGyroValueToMachineRadian((int32_t)phi);
					x+=-(float)(vl+vr)/2.f*MyMath::sin(phiInRadian);
					y+=(float)(vl+vr)/2.f*MyMath::cos(phiInRadian);
				}
				void calculateOutput(float &outputL,float &outputR) volatile{ outputL=v-w; outputR=v+w; }
			};
			volatile State state;

			Trajectory::TargetSequence targetSequence;
			int32_t lastVelocity;

			Vert::PIDController<float> angleController;
			float wallKp;
			float wallKd;
			int16_t gyr1X, gyr1Y, gyr1Z, acc1X, acc1Y, acc1Z;
			int16_t gyr2X, gyr2Y, gyr2Z, acc2X, acc2Y, acc2Z;
			float gyr1X_offs, gyr2X_offs;

			volatile uint16_t adcValues_lit1[6], adcValues_lit2[6], adcValues_unlit[6];
			volatile uint16_t adcValues[6];
			volatile uint16_t adcNeutralLF, adcNeutralRF, adcNeutralFR, adcNeutralBAT;
			uint16_t adcMinRF;
			volatile uint16_t adcThresholdL, adcThresholdR, adcThresholdFR;

		public:
			Machine(): htim_{}, tickCounter(0), isActivated_(false), isBlocked_(true),
				motorLController(120.f, 18879.f, 0.f, dt*2, false),
				motorRController(110.f, 18837.f, 0.f, dt*2, false),
				state{},
				targetSequence(0.001f),
				lastVelocity(0),
				angleController(0.00065f, 0.050f, 0.f, dt*4, false),
				wallKp(3.f), wallKd(2300.f),
				gyr1X(0),gyr1Y(0),gyr1Z(0),acc1X(0),acc1Y(0),acc1Z(0),
				gyr2X(0),gyr2Y(0),gyr2Z(0),acc2X(0),acc2Y(0),acc2Z(0),
				gyr1X_offs(0),gyr2X_offs(0),
				adcValues_lit1{}, adcValues_lit2{}, adcValues_unlit{},
				adcValues{},
				adcNeutralLF(0),adcNeutralRF(0),adcNeutralFR(0),adcNeutralBAT(0),
				adcMinRF(1500),
				adcThresholdL(1300), adcThresholdR(2400), adcThresholdFR(3250), // neutral + 400, 400, 200
				log{},isLogging(false),writePos(0)
			{
				TIM_ClockConfigTypeDef sClockSourceConfig;
				TIM_MasterConfigTypeDef sMasterConfig;

				__HAL_RCC_TIM5_CLK_ENABLE();

				// 4kHz
				htim_.Instance = TIM5;
				htim_.Init.Prescaler = 24;
				htim_.Init.CounterMode = TIM_COUNTERMODE_UP;
				htim_.Init.Period = 999;
				htim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
				if (HAL_TIM_Base_Init(&htim_) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
				if (HAL_TIM_ConfigClockSource(&htim_, &sClockSourceConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(&htim_, &sMasterConfig) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

				HAL_TIM_Base_Start_IT(&htim_);
			}

			void configIRQ(){
				HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
				HAL_NVIC_EnableIRQ(TIM5_IRQn);
			}

			void activate(){ isActivated_ = true; }
			void deactivate(){ isActivated_ = false; }
			bool isActivated() const{ return isActivated_; }

			void block(){ isBlocked_ = true; }
			void unblock(){ isBlocked_ = false; }
			bool isBlocked() const{ return isBlocked_; }

			void resetTick(){
				tickCounter = 0;
			}

			void onTimerUpdate() {
				HAL_TIM_IRQHandler(&htim_);

				tickCounter++;

				switch(tickCounter%4){
					case 0:
						buzzer.update();
						break;
					case 1:
						leds.update();
						break;
					case 2:
					case 3:
						break;
				}

				if(isBlocked()){
					motors.setOutput(0,0);
					irled1.stopPulse();
					irled2.stopPulse();
					return;
				}

				switch(tickCounter%4){
					case 0:
						motorControl();
						irled1.stopPulse();
						irled2.stopPulse();
						irsensor.startConv();
						break;
					case 1:
						//while(!irsensor.isCompleted()){}
						irsensor.stopConv();
						irsensor.resetCompleted();
						irsensor.readValues(adcValues_unlit);
						irled1.triggerPulse();
						irsensor.startConv();
						break;
					case 2:
						motorControl();
						pollIMUValues();
						//while(!irsensor.isCompleted()){}
						irsensor.stopConv();
						irled1.stopPulse();
						irsensor.resetCompleted();
						irsensor.readValues(adcValues_lit1);
						irled2.triggerPulse();
						irsensor.startConv();
						break;
					case 3:
						irsensor.stopConv();
						irled2.stopPulse();
						irsensor.resetCompleted();
						irsensor.readValues(adcValues_lit2);
						updateADCValues();
						poseControl();
						break;
				}
			}

			void motorControl(){
				if(!isActivated()){ motors.stop(); return; }
				// TODO: apply smoothing filters to observed values
				float outputL, outputR;
				state.calculateOutput(outputL, outputR);
				encoderL.captureSpeed(); encoderR.captureSpeed();
				float vL = encoderL.speed(), vR = -encoderR.speed();
				motorLController.update(outputL, vL); motorRController.update(outputR, vR);
				float pwmL = motorLController.getOutput(), pwmR = motorRController.getOutput();
				motors.setOutput(pwmL, pwmR);

				if(isLogging){
					log[writePos++] = vL; log[writePos++] = vR; log[writePos++] = pwmL; log[writePos++] = pwmR;
					if(writePos>=logLength){ isLogging=false; }
				}
				state.update(vL, vR);
			}

			void poseControl(){
				static uint8_t noTargetCount = 0;
				static uint8_t lowBatteryCount = 0;
				static bool targetUpdated = false;
				if(!isActivated()){
					motors.stop();
					noTargetCount = 0;
					lowBatteryCount = 0;
					targetUpdated = false;
					return;
				}

				using namespace Trajectory;
				using namespace MyMath;
				using namespace MyMath::Machine;

				float wallAdj = -wallKd*(state.v+2.f)*sin(convertGyroValueToMachineRadian(state.phi-state.rphi));
				if(fabs(state.v)>2.f && fabs(state.w)<5.f && (int16_t)(convertGyroValueToMachineDegree(state.rphi))%90==0 ){
					if(adcValues[IRSensor::LF]>adcThresholdL && adcValues[IRSensor::RF]>adcThresholdR){
						wallAdj += -wallKp*signof(state.v)*(adcValues[IRSensor::LF]-adcNeutralLF-adcValues[IRSensor::RF]+adcNeutralRF);
					} else if(adcValues[IRSensor::LF]>adcThresholdL && adcValues[IRSensor::RF]<=adcThresholdR){
						wallAdj += -2.f*wallKp*signof(state.v)*(adcValues[IRSensor::LF]-adcNeutralLF);
					} else if(adcValues[IRSensor::RF]>adcThresholdR && adcValues[IRSensor::LF]<=adcThresholdL){
						wallAdj += -2.f*wallKp*signof(state.v)*(-adcValues[IRSensor::RF]+adcNeutralRF);
					}
				}

				if(!targetSequence.isEmpty()){
					targetUpdated = true;
					noTargetCount = 0;

					Position p,v;
					targetSequence.incrementTimestamp();
					p=convertRealPositionToMachinePosition(targetSequence.getCurrentPosition());
					v=convertRealPositionToMachinePosition(targetSequence.getCurrentVelocity());
					targetSequence.nextTarget();

					setReference(p.x, p.y, p.angle);
					setReferenceVelocity(v.x, v.y, v.angle);

					float sgn = signof(-sin(convertGyroValueToMachineRadian(state.rphi))*v.x+cos(convertGyroValueToMachineRadian(state.rphi))*v.y);

					// Here, v is the pulse count of encoders in 1ms (time period of the moving target).
					// Motor controllers are updated every 0.5ms, so the value should be divided by 2.
					state.v = sgn * sqrt(v.x*v.x+v.y*v.y) * 0.5f;

					angleController.update(v.angle+wallAdj, state.wsens);
					state.w = angleController.getOutput();
				} else {
					targetUpdated = false;
					if(isActivated()) {
						noTargetCount++;
						state.v = 0;
						if(noTargetCount>50) {
							noTargetCount=50;
							state.w = 0;
						} else {
							angleController.update(wallAdj, state.wsens);
							state.w = angleController.getOutput();
						}
					} else {
						state.v = 0;
						state.w = 0;
					}
				}

				// FAILSAFE
				float dphi = state.rphi-state.phi;
				MyMath::normalize(dphi, PIInGyroValue);
				if(MyMath::fabs(dphi) >= PIInGyroValue/4.f) {
					deactivate();
				}

				// display wall detection
				for (uint8_t i = 0; i < 6; ++i) {
					leds.reset(i);
				}
				if(adcValues[IRSensor::LF] > adcThresholdL){ leds.set(0); }
				if(adcValues[IRSensor::RF] > adcThresholdR){ leds.set(1); }
				if(adcValues[IRSensor::FR] > adcThresholdFR){ leds.set(2); leds.set(3); }

				if(adcValues[IRSensor::BAT] < 2150) {
					lowBatteryCount++;
					if(lowBatteryCount > 200) {
						//for(auto led : dleds) led.reset();
						//dleds[3].set();
						deactivate();
					}
				} else {
					lowBatteryCount = 0;
				}
			}

			void pollIMUValues(){
				if(!isActivated()){ motors.stop(); return; }
				imu1.readGyrXYZ(gyr1X,gyr1Y,gyr1Z);
				imu2.readGyrXYZ(gyr2X,gyr2Y,gyr2Z);
				state.wsens = -(float)((gyr1X-gyr1X_offs)+(gyr2X-gyr2X_offs))/2.f;
			}

			void refreshIMUOffsets(){
				bool tmp_blocked = isBlocked();
				if(!tmp_blocked) block();

				imu1.reset();
				imu2.reset();
				HAL_Delay(500);
				imu1.writeInitReg();
				imu2.writeInitReg();
				HAL_Delay(500);

				int32_t sum1 = 0;
				int32_t sum2 = 0;
				for (uint16_t i = 0; i < 600; ++i) {
					int16_t offs1X, offs1Y, offs1Z = 0;
					int16_t offs2X, offs2Y, offs2Z = 0;
					imu1.readGyrXYZ(offs1X, offs1Y, offs1Z);
					imu2.readGyrXYZ(offs2X, offs2Y, offs2Z);
					sum1 += offs1X;
					sum2 += offs2X;
					HAL_Delay(2);
				}
				gyr1X_offs = (float)sum1/600.f;
				gyr2X_offs = (float)sum2/600.f;

				if(!tmp_blocked) unblock();
			}

			void updateADCValues(){
				for (uint8_t i = 0; i < 6; ++i) {
					uint16_t litVal;
					if(i == IRSensor::LF || i == IRSensor::RF){
						litVal = adcValues_lit1[i];
					} else {
						litVal = adcValues_lit2[i];
					}
					if(adcValues_unlit[i] > litVal){
						adcValues[i]=0;
					} else {
						using namespace MyMath;
						if(MyMath::Machine::CellWidth==90.f){
							// Half-size
							adcValues[i] = (uint16_t)(MyMath::sqrt((float)litVal - (float)adcValues_unlit[i])*64.f);
						} else if(MyMath::Machine::CellWidth == 180.f) {
							// Classic
							adcValues[i] = (uint16_t)(MyMath::sqrt((float)litVal - (float)adcValues_unlit[i])*90.f);
						} else {
							// Quarter-size
							adcValues[i] = (uint16_t)(MyMath::sqrt((float)litVal - (float)adcValues_unlit[i])*64.f);
						}
					}
				}
				// for battery
				adcValues[IRSensor::BAT] = (adcValues_lit1[IRSensor::BAT] + adcValues_lit2[IRSensor::BAT] + adcValues_unlit[IRSensor::BAT])/3;
			}

			uint16_t getADCValue(uint8_t index) const{
				return adcValues[index];
			}

			void setNeutralSideSensorValue(){
				uint32_t sumLF=0, sumRF=0, sumFR=0, sumBAT=0;
				for(uint8_t i=0; i<50; i++){
					sumLF += adcValues[IRSensor::LF];
					sumRF += adcValues[IRSensor::RF];
					sumFR += adcValues[IRSensor::FR];
					sumBAT += adcValues[IRSensor::BAT];
					HAL_Delay(5);
				}
				adcNeutralLF = sumLF / 50;
				adcNeutralRF = sumRF / 50;
				if(adcNeutralRF < adcMinRF) {
					adcNeutralRF = adcNeutralLF;
				}
				adcNeutralFR = sumFR / 50;
				adcNeutralBAT = sumBAT / 50;

				if(MyMath::Machine::CellWidth == 90.f){
					// Half-size
					adcThresholdL = adcNeutralLF - 200;
					adcThresholdR = adcNeutralRF - 200;
					adcThresholdFR = adcNeutralFR + 130;
				} else if(MyMath::Machine::CellWidth == 180.f) {
					// Classic
					adcThresholdL = adcNeutralLF - 100;
					adcThresholdR = adcNeutralRF + 100;
					adcThresholdFR = adcNeutralFR + 200;
				} else {
					// Quarter-size
					adcThresholdL = adcNeutralLF - 400;
					adcThresholdR = adcNeutralRF - 350;
					adcThresholdFR = adcNeutralFR + 630;
				}
			}

			bool isSetWall(uint8_t dir){
				float ratio = ((float)adcValues[IRSensor::BAT]/(float)adcNeutralBAT);
				buzzer.addNote(0,1);
				buzzer.addNote('c', 5, 20);
				buzzer.play();
				if((dir&0xf)==0x1){
					return (adcValues[IRSensor::FR]>adcThresholdFR);
				} else if((dir&0xf)==0x2){
					return (adcValues[IRSensor::RF]>adcThresholdR);
				} else if((dir&0xf)==0x8){
					return (adcValues[IRSensor::LF]>adcThresholdL);
				}
				return false;
			}

			void resetControllers(){
				motorLController.reset();
				motorRController.reset();
				angleController.reset();

				std::fill(adcValues, adcValues+6, 0);
				std::fill(adcValues_unlit, adcValues_unlit+6, 0);
				std::fill(adcValues_lit1, adcValues_lit1+6, 0);
				std::fill(adcValues_lit2, adcValues_lit2+6, 0);
			}

			void setReference(float rx,float ry,float rphi) volatile{
				using namespace MyMath;
				using namespace MyMath::Machine;
				state.rx=rx;
				state.ry=ry;
				normalize(rphi, PIInGyroValue);
				state.rphi=rphi;
			}

			void setReferenceVelocity(float rvx,float rvy,float rw) volatile{
				state.rvx=rvx;
				state.rvy=rvy;
				state.rw=rw;
			}
			void setState(float x,float y,float phi) volatile{
				state.x=MyMath::Machine::convertWheelDistanceToPulse(x);
				state.y=MyMath::Machine::convertWheelDistanceToPulse(y);
				state.phi=MyMath::Machine::convertMachineRadianToGyroValue(phi);
				state.rx=state.x;
				state.ry=state.y;
				state.rphi=state.phi;
				state.v=0;
				state.w=0;
				state.rvx=0;
				state.rvy=0;
				state.rw=0;
				state.wsens=0;
			}
			void setStateXYToReference() volatile{
				state.x = state.rx;
				state.y = state.ry;
			}

			uint16_t getLastVelocity() const { return lastVelocity; }
			Trajectory::Position getLastPosition() const { return targetSequence.getLastPosition(); }

			void pushTargetDiff(const Trajectory::Position& diff, Trajectory::MotionFunctor *mf, Trajectory::Parameters p){
				targetSequence.pushDifference(diff, mf, p);
				lastVelocity=p.vf;
			}

			void pushTarget(const Trajectory::Position& pos, Trajectory::MotionFunctor* mf, Trajectory::Parameters p){
				targetSequence.push(pos, mf, p);
				lastVelocity=p.vf;
			}

			bool isTargetSequenceEmpty() const{
				return targetSequence.isEmpty();
			}

			void resetTargetSequence(const Trajectory::Position& pos){
				targetSequence.resetSequence(pos);
				lastVelocity=0;
			}

			void startLogging(){ isLogging = true; }
			void stopLogging(){ isLogging = false; }

			constexpr static uint16_t logLength = 6000;
			float log[logLength];
			bool isLogging;
			uint16_t writePos;
	};
}
