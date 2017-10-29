#pragma once

#include <cmath>
#include "astolfi.h"
#include "pidcontroller.h"
#include "mymath.h"
#include "mymath_machine.h"

namespace Vert{
	template <class TPlant> class VelocityWithAstolfi : public Astolfi<TPlant> {
		public:
			VelocityWithAstolfi(float Krho,float Kalpha,float Kphi,float vsat,float wsat,float Kpv,float Kpw):Astolfi<TPlant>(Krho,Kalpha,Kphi,vsat,wsat),pid_v(Kpv,0,0),pid_w(Kpw,0,0){}
			~VelocityWithAstolfi(){}
			int8_t control(TPlant &plant){
				int8_t result = Astolfi<TPlant>::control(plant);
				if(result<0) return -1;
				using namespace MyMath;
				using namespace MyMath::Machine;
				pid_v.update(0.2f*sqrt(plant.rvx*plant.rvx+plant.rvy*plant.rvy), plant.v);
				pid_w.update(-0.1f*plant.rw, plant.w);
				plant.v+=pid_v.getOutput();
				plant.w+=pid_w.getOutput();
				return 0;
			}
			void setGain(float _Krho, float _Kalpha, float _Kphi, float _vsat, float _wsat, float _Kpv, float _Kpw){
				Astolfi<TPlant>::setGain(_Krho, _Kalpha, _Kphi, _vsat, _wsat);
				pid_v.setGain(_Kpv,0.f,0.f,0.f);
				pid_w.setGain(_Kpw,0.f,0.f,0.f);
			}
		private:
			PIDController<float> pid_v;
			PIDController<float> pid_w;
	};
}
