#pragma once

#include "position.h"
#include "functors.h"

namespace Trajectory {

	class Target {
		private:
			unsigned short time;
			MotionFunctor *mf;

		public:
			Target(MotionFunctor *_mf,float _dt):mf(_mf){
				time=(unsigned short)(mf->getTime()/_dt + 0.5f);
			}
			void destroy(){ mf->destroy(); delete mf; }

			Position getCurrentPosition(unsigned short t) const {
				float s=(float)t/(float)time;
				return mf->func(s);
			}

			Position getCurrentVelocity(unsigned short t) const {
				float s=(float)t/(float)time;
				return mf->vfunc(s)/(float)time;
			}

			MotionFunctor *getMF() const { return mf; }

			unsigned short getTime() const { return time; }
	};

}
