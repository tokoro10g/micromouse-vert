#include "pidcontroller.h"

namespace Vert{
	template<typename T, class TMotor, class TEncoder>
		class MotorController {
			public:
				MotorController(T Kp,T Ki,T Kd,T Kff,bool reseti=true,T isat=0):TMotor(),TEncoder(),pid(Kp,Ki,Kd,reseti,isat),_Kff(Kff){}
				void setGain(T Kp,T Ki,T Kd,T isat=0){
					pid.setGain(Kp,Ki,Kd,isat);
				}
				inline void reset(){
					pid.reset();
				}
				int16_t update(T r, T x){
					static uint8_t overCnt=0;
					pid.update(r,x);
					if(pid.getIntegrator()>2000){
						overCnt++;
					} else {
						overCnt=0;
					}
					//if(overCnt>70){ throw -1; }
					return -pid.getOutput()-_Kff*r;
				}
			private:
				PIDController<T> pid;
				T _Kff;
		};
}
