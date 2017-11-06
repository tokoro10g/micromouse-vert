#pragma once
#include "mymath.h"

namespace Vert{
	template<typename T>
		class PIDController {
			public:
				PIDController(T _Kp, T _Ki, T _Kd, T _Ts, bool _reseti=true, T _isat=T(0)):Kp(_Kp),Ki(_Ki),Kd(_Kd),Ts(_Ts),reseti(_reseti),isat(_isat),ep(T(0)),ei(T(0)),u(T(0)){}
				~PIDController(){}
				void update(T r, T x){
					if(reseti && r!=rp) ei=T(0);
					T e=r-x; T ed=(e-ep)/Ts; ei+=e*Ts; ep=e;
					if(isat>T(0)){ MyMath::saturate(ei, isat); } // Anti wind-up
					u=Kp*e+Ki*ei+Kd*ed;
					rp=r;
				}
				void reset(){
					u=T(0); ep=T(0); ei=T(0);
				}
				void setGain(T _Kp,T _Ki,T _Kd,T _isat){
					Kp=_Kp; Ki=_Ki; Kd=_Kd; isat=_isat;
				}
				T getIntegrator() const { return ei; }
				T getOutput() const { return u; }
			private:
				T Kp;
				T Ki;
				T Kd;
				T Ts;
				bool reseti;
				T isat;
				T ep;
				T ei;
				T u;
				T rp;
		};
}
