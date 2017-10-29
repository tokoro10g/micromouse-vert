#pragma once

#include <cmath>
#include "mymath.h"
#include "mymath_machine.h"

namespace Vert{
	template <class TPlant> class Astolfi {
		public:
			Astolfi(float Krho,float Kalpha,float Kphi,float vsat,float wsat):Krho(Krho),Kalpha(Kalpha),Kphi(Kphi),vsat(vsat),wsat(wsat){}
			~Astolfi(){}
			int8_t control(TPlant &plant){
				using namespace MyMath;
				using namespace MyMath::Machine;
				float ex=plant.rx-plant.x;
				float ey=plant.ry-plant.y;
				float ephi=plant.rphi-plant.phi;

				float erho=sqrt(ex*ex+ey*ey);
				float ealpha;
				if(erho<400){
					// target point is close to center of the machine
					erho=0;
					ealpha=ephi;
				} else {
					// avoid chattering
					erho-=300;
					// target point is far from center of the machine
					ealpha=convertMachineRadianToPulseDiff(PI/2.0-atan2(ey,ex))-plant.phi;
					normalize(ealpha, PIInPulseDiff);
					if(fabs(ealpha)>=PIInPulseDiff/1.5f){
						// target point is behind the machine
						erho=-erho;
					}
					normalize(ealpha, PIInPulseDiff/2.f);
					if(fabs(erho)<1000){
						ealpha*=fabs(erho)/1000;
					}
				}
				if(fabs(ephi)<100){
					ephi=0;
				} else {
					ephi-=signof(ephi)*100;
				}
				if(fabs(ealpha)<100){
					ealpha=0;
				} else {
					ealpha-=signof(ealpha)*100;
				}

				normalize(ephi, PIInPulseDiff);
				normalize(ealpha, PIInPulseDiff);

				plant.v=plant.v*0.5+saturated(Krho*erho, vsat)*0.5;
				plant.w=plant.w*0.5+saturated(Kalpha*ealpha+Kphi*ephi, wsat)*0.5;

				if(fabs(ephi)>convertMachineRadianToPulseDiff(PI/2.f)){
					return -1;
				}
				return 0;
			}
			void setGain(float _Krho, float _Kalpha, float _Kphi, float _vsat, float _wsat){
				Krho=_Krho; Kalpha=_Kalpha; Kphi=_Kphi; vsat=_vsat; wsat=_wsat;
			}
		private:
			float Krho;
			float Kalpha;
			float Kphi;
			float vsat;
			float wsat;
	};
}
