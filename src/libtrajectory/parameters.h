#pragma once

namespace Trajectory {
	struct Parameters {
		float v0;
		float vf;
		float vmax;
		float amax;
		Parameters():v0(0),vf(0),vmax(0),amax(0){}
		Parameters(float _v0,float _vf,float _vmax,float _amax):v0(_v0),vf(_vf),vmax(_vmax),amax(_amax){}
		Parameters(const Parameters& p){
			v0=p.v0; vf=p.vf; vmax=p.vmax; amax=p.amax;
		}
	};
	/*
	static const Parameters p_straight_start(0,800,2000,10000);
	static const Parameters p_straight(800,800,2000,10000);
	static const Parameters p_straight_end(800,0,2000,10000);
	static const Parameters p_turn(800,800,1000,2000);
	*/

}
