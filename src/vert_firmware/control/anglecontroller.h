#include "mymath.h"
#include "mymath_machine.h"
#include "pidcontroller.h"

namespace Vert{
	template<typename T> class AngleController {
		public:
			AngleController(T Kp,T Ki,T Kd,bool reseti=true,T isat=0):pid(Kp,Ki,Kd,reseti,isat),u(T(0)),rp(T(0)),drp(T(0)){}
			~AngleController(){}
			void setGain(T Kp,T Ki,T Kd,T isat=0){
				pid.setGain(Kp,Ki,Kd,isat);
			}
			inline void reset(){
				pid.reset();
				u=0; rp=0; drp=0;
			}
			void update(T r, T x){
				float dr = r-rp;
				MyMath::normalize(dr, MyMath::Machine::PIInGyroValue);
				float ddr = dr-drp;
				MyMath::normalize(ddr, MyMath::Machine::PIInGyroValue);
				rp = r;
				drp = dr;

				float e = r-x;
				MyMath::normalize(e, MyMath::Machine::PIInGyroValue);

				pid.update(e,0);
				//u = pid.getOutput() + 0.0281f*dr + 0.035*ddr;
				//u = pid.getOutput() + 1.51f*MyMath::signof(dr)*MyMath::sqrt(MyMath::fabs(dr));
				u = pid.getOutput() + 0.052f*dr;
				//u = pid.getOutput();
			}
			T getOutput() const {
				return u;
			}
		private:
			PIDController<T> pid;
			T u;
			T rp;
			T drp;
	};
}
