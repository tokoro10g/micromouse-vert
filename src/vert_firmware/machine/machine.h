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
				float x, y, phi, alpha, rx, ry, rphi, rvx, rvy, rw, v, w, wsens;
				State():x(0),y(0),phi(0),alpha(0),rx(0),ry(0),rphi(0),rvx(0),rvy(0),rw(0),v(0),w(0),wsens(0){}
				void update(int16_t vl, int16_t vr) volatile{
					phi+=wsens; phi = MyMath::normalized(phi, MyMath::Machine::PIInGyroValue);
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
			int16_t gyr1X, gyr1Y, gyr1Z, acc1X, acc1Y, acc1Z;
			int16_t gyr2X, gyr2Y, gyr2Z, acc2X, acc2Y, acc2Z;
			float gyr1X_offs, gyr2X_offs;

			uint16_t adcValues_lit[6], adcValues_unlit[6];
			volatile uint16_t adcValues[6];
			uint16_t adcNeutralLF, adcNeutralRF, adcNeutralBAT;
			uint16_t adcMinRF;
			uint16_t adcThresholdL, adcThresholdR, adcThresholdFR;

		public:
			Machine(): htim_{}, tickCounter(0), isActivated_(false), isBlocked_(true),
				motorLController(110.f, 18879.f, 0.f, dt*2, false),
				motorRController(100.f, 18737.f, 0.f, dt*2, false),
				state{},
				targetSequence(0.001f),
				lastVelocity(0),
				angleController(0.00065f, 0.050f, 0.000001f, dt*4, false),
				gyr1X(0),gyr1Y(0),gyr1Z(0),acc1X(0),acc1Y(0),acc1Z(0),
				gyr2X(0),gyr2Y(0),gyr2Z(0),acc2X(0),acc2Y(0),acc2Z(0),
				gyr1X_offs(0),gyr2X_offs(0),
				adcValues_lit{}, adcValues_unlit{},
				adcValues{},
				adcNeutralLF(0),adcNeutralRF(0),adcNeutralBAT(0),
				adcMinRF(0),
				adcThresholdL(1000), adcThresholdR(1000), adcThresholdFR(3000),
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

				if(isBlocked()){ return; }

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
						irled2.triggerPulse();
						irsensor.startConv();
						break;
					case 2:
						motorControl();
						pollIMUValues();
						//while(!irsensor.isCompleted()){}
						irsensor.stopConv();
						irled1.stopPulse();
						irled2.stopPulse();
						irsensor.resetCompleted();
						irsensor.readValues(adcValues_lit);
						updateADCValues();
						break;
					case 3:
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
					state.v = sgn * sqrt(v.x*v.x+v.y*v.y) * 0.5f;

					angleController.update(v.angle, state.wsens);
					state.w = angleController.getOutput();
				} else {
					targetUpdated = false;
					if(isActivated()) {
						noTargetCount++;
						if(noTargetCount>50) {
							noTargetCount=50;
							state.v=0;
						}
					}
					// FIXME
					state.w = 0;
				}

				// FAILSAFE
				float dphi = state.rphi-state.phi;
				MyMath::normalize(dphi, PIInGyroValue);
				if(MyMath::fabs(dphi) >= PIInGyroValue/4.f) {
					//for(auto led : dleds) led.reset();
					//dleds[2].set();
					deactivate();
				}

/*
#ifdef MAZEDEBUG
				state.x = state.rx;
				state.y = state.ry;
				state.phi = state.rphi;
#endif
*/

				// display wall detection
				//dleds[0].reset(); dleds[1].reset(); dleds[2].reset();

				//TODO: wall correction

				//bool frontCorrection = false;

				//float sinphi = MyMath::sin(convertGyroValueToMachineRadian(state.phi));
				//float cosphi = MyMath::cos(convertGyroValueToMachineRadian(state.phi));
				if(targetUpdated && MyMath::fabs(state.v)>10){
					//state.w-=adj*signof(state.v);
					//if(positionCorrection) {
					//	state.x-=adj*cos(convertGyroValueToMachineRadian(state.rphi));
					//	state.y-=adj*sin(convertGyroValueToMachineRadian(state.rphi));
					//}
					//transverseCtrl.update(0, signof(state.v)*((state.rx-state.x)*cosphi+(state.ry-state.y)*sinphi));
					//state.w += transverseCtrl.getOutput();
					//verticalCtrl.update(0, (state.rx-state.x)*sinphi-(state.ry-state.y)*cosphi);
					//state.v += verticalCtrl.getOutput();
				} else {
					//verticalCtrl.update(0, (state.rx-state.x)*sinphi-(state.ry-state.y)*cosphi);
					//state.v = verticalCtrl.getOutput();
				}

				//if(adcValues[IRSensor::BAT] < 2150) {
				//	lowBatteryCount++;
				//	if(lowBatteryCount > 200) {
				//		//for(auto led : dleds) led.reset();
				//		//dleds[3].set();
				//		deactivate();
				//	}
				//} else {
				//	lowBatteryCount = 0;
				//}
			}

			void pollIMUValues(){
				if(!isActivated()){ motors.stop(); return; }
				imu1.readGyrXYZ(gyr1X,gyr1Y,gyr1Z);
				imu2.readGyrXYZ(gyr2X,gyr2Y,gyr2Z);
				state.wsens = -(float)((gyr1X-gyr1X_offs)+(gyr2X-gyr2X_offs))/2.f;
			}

			void refreshIMUOffsets(){
				// TODO: reset IMUs before reading data to perform internal calibration
				/*
				// this causes unexpected behavior :(
				imu1.reset();
				imu2.reset();
				HAL_Delay(1000);
				*/
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
			}

			void updateADCValues(){
				for (uint8_t i = 0; i < 6; ++i) {
					if(adcValues_unlit[i] > adcValues_lit[i]){
						adcValues[i]=0;
					} else {
						adcValues[i] = adcValues_lit[i] - adcValues_unlit[i];
					}
				}
				// for battery
				adcValues[4] = (adcValues_lit[4] + adcValues_unlit[4])/2;
			}

			uint16_t getADCValue(uint8_t index) const{
				return adcValues[index];
			}

			void setNeutralSideSensorValue(){
				uint32_t sumLF=0, sumRF=0, sumBAT=0;
				for(uint8_t i=0; i<20; i++){
					sumLF += adcValues[IRSensor::LF];
					sumRF += adcValues[IRSensor::RF];
					sumBAT += adcValues[IRSensor::BAT];
					HAL_Delay(5);
				}
				adcNeutralLF = sumLF / 20;
				adcNeutralRF = sumRF / 20;
				if(adcNeutralRF < adcMinRF) {
					adcNeutralRF = adcNeutralLF;
				}
				adcNeutralBAT = sumBAT / 20;
			}

			bool isSetWall(uint8_t dir){
				float ratio = ((float)adcValues[IRSensor::BAT]/(float)adcNeutralBAT);
				if((dir&0xf)==0x1){
					return (adcValues[IRSensor::FR]>ratio*adcThresholdFR);
				} else if((dir&0xf)==0x2){
					return (adcValues[IRSensor::RF]>ratio*adcThresholdR);
				} else if((dir&0xf)==0x8){
					return (adcValues[IRSensor::LF]>ratio*adcThresholdL);
				}
				return false;
			}

			void resetControllers(){
				motorLController.reset();
				motorRController.reset();
				angleController.reset();

				std::fill(adcValues, adcValues+6, 0);
				std::fill(adcValues_unlit, adcValues_unlit+6, 0);
				std::fill(adcValues_lit, adcValues_lit+6, 0);
			}

			void temp_setStatev(float v){
				state.v = v;
			}
			void temp_setStaterw(float rw){
				state.rw = rw;
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
				state.alpha=0;
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
			}

			void startLogging(){ isLogging = true; }
			void stopLogging(){ isLogging = false; }

			constexpr static uint16_t logLength = 6000;
			float log[logLength];
			bool isLogging;
			uint16_t writePos;
	};
}
