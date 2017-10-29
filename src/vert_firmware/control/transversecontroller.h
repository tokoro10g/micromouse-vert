#include "mymath.h"
#include "mymath_machine.h"
#include "pidcontroller.h"

namespace Vert{
	template<typename T> class TransverseController {
		public:
			TransverseController(T Kp,T Ki,T Kd,bool reseti=true,T isat=0):pid(Kp,Ki,Kd,reseti,isat),u(T(0)){}
			~TransverseController(){}
			void setGain(T Kp,T Ki,T Kd,T isat=0){
				pid.setGain(Kp,Ki,Kd,isat);
			}
			inline void reset(){
				pid.reset();
				u=0;
			}
			void update(T r, T x){
				pid.update(r,x);
				u = pid.getOutput();
			}
			T getOutput() const {
				return u;
			}
		private:
			PIDController<T> pid;
			T u;
	};
}
