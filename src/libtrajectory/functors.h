#pragma once

#include "position.h"
#include "mymath.h"
#include <algorithm>

namespace Trajectory {

	/*
	 * Base functors
	 * */

	class EasingFunctor{
		public:
			EasingFunctor(){}
			virtual ~EasingFunctor(){}
			virtual float func(float s) const = 0;
			virtual float vfunc(float s) const = 0;
			virtual void initialise(){}
			virtual float configureParams(float v0,float vf,float vmax,float amax) = 0;
	};

	class MotionFunctor{
		protected:
			Position origin;
			Position dest;
			EasingFunctor *e;
			float time;
		public:
			MotionFunctor(){}
			MotionFunctor(Position _origin,Position _dest,EasingFunctor *_e):origin(_origin),dest(_dest),e(_e){}
			MotionFunctor(EasingFunctor *_e):e(_e){}
			MotionFunctor(const MotionFunctor& obj){
				origin=obj.origin; dest=obj.dest; e=obj.e;
			}
			virtual ~MotionFunctor(){}
			void destroy(){ delete e; }

			float getTime() const{ return time; }
			Position getOrigin() const { return origin; }
			Position getDest() const { return dest; }
			EasingFunctor *getE() const { return e; }
			void setPosition(Position _origin,Position _dest){
				origin=_origin;
				dest=_dest;
			}
			virtual void configureParams(float v0,float vf,float vmax,float amax){
				float d=distance();
				time=e->configureParams(v0/d,vf/d,vmax/d,amax/d);
			}
			virtual Position func(float s) = 0;
			virtual Position vfunc(float s) = 0;
			virtual void initialise(){}
			virtual float distance() const = 0;
	};

	/*
	 * Derived functors
	 * */

	class EasingPoly5 : public EasingFunctor{
		private:
			float a,b,c,d,e;
		public:
			EasingPoly5():EasingFunctor(){}
			EasingPoly5(const EasingPoly5& obj){
				a=obj.a; b=obj.b; c=obj.c; d=obj.d; e=obj.e;
			}
			~EasingPoly5(){}
			float func(float s) const{
				return a*s*s*s*s*s+b*s*s*s*s+c*s*s*s+d*s*s+e*s;
			}
			float vfunc(float s) const{
				return 5.f*a*s*s*s*s+4.f*b*s*s*s+3.f*c*s*s+2.f*d*s+e;
			}
			float configureParams(float v0,float vf,float vmax,float amax){
				float t=1.875f/(vmax+0.363f*(v0+vf));
				float v1=std::min(vf,v0), v2=std::max(vf,v0);
				if((5.77f/t-3.82f*v1-1.83f*v2)/t>amax){
					float temp = 3.82f*v1+1.83f*v2;
					t=(MyMath::sqrt(577.f*amax+25.f*temp*temp)-5.f*temp)/(10.f*amax);
				}
				float ev0=v0*t; float evf=vf*t;
				a=-3.f*(ev0+evf)+6.f;
				b=8.f*ev0+7.f*evf-15.f;
				c=-6.f*ev0-4.f*evf+10.f;
				d=0;
				e=ev0;
				return t;
			}
	};

	class EasingTrap : public EasingFunctor{
		private:
			float t,ta,tb;
			float ev0, evf, evmax, eamax, evstar;
		public:
			EasingTrap():EasingFunctor(){}
			~EasingTrap(){}
			float func(float s) const{
				float p;
				if(s < ta/t){
					p = ev0*s + eamax*s*s/2.f;
				} else if(s < 1.f-tb/t) {
					p = ev0*ta/t + eamax*ta*ta/t/t/2.f + evstar*(s-ta/t);
				} else {
					p = ev0*ta/t + eamax*ta*ta/t/t/2.f + evstar*(1.f-tb/t-ta/t) + evstar*(s-1.f+tb/t) - eamax * ((s-1.f+tb/t)*(s-1.f+tb/t)/2.f);
				}
				return p;
			}
			float vfunc(float s) const{
				float v;
				if(s < ta/t){
					v = ev0 + eamax * s;
				} else if(s < 1.f-tb/t) {
					v = evstar;
				} else {
					v = evstar - eamax * (s-1.f+tb/t);
				}
				return v;
			}
			float configureParams(float v0,float vf,float vmax,float amax){
				float vstar;
				ta = (vmax-v0)/amax;
				tb = (vmax-vf)/amax;
				if(ta*(vmax+v0)+tb*(vmax+vf) > 2.f){
					vstar = MyMath::sqrt(amax+v0*v0/2.f+vf*vf/2.f);
					if(MyMath::fabs(vstar-v0) < 1e-5){
						/* |\
						 * || */
						t = (v0-vf)/amax; ta = 0; tb = t;
					} else if(MyMath::fabs(vstar-vf) < 1e-5){
						/* /|
						 * || */
						t = (vf-v0)/amax; ta = t; tb = t;
					} else {
						/* /\
						 * || */
						t = (2.f*vstar-v0-vf)/amax; ta = (vstar-v0)/amax; tb = (vstar-vf)/amax;
					}
				} else {
					/* /~~\
					 * |  | */
					vstar = vmax; t = ta+tb+(1.f-(ta*(vmax+v0)+tb*(vmax+vf))/2.f)/vmax;
				}
				ev0 = v0*t, evf = vf*t, evmax = vmax*t, eamax = amax*t*t;
				evstar = vstar*t;
				return t;
			}
	};

	class EasingLinear : public EasingFunctor{
		public:
			EasingLinear():EasingFunctor(){}
			~EasingLinear(){}
			float func(float s) const{
				return s;
			}
			float vfunc(float s) const{
				return 1.f;
			}
			float configureParams(float v0,float vf,float vmax,float amax){
				return 1.f/v0;
			}
	};

	class MotionLinear : public MotionFunctor{
		private:
			Position diff;
		public:
			MotionLinear(EasingFunctor *_e):MotionFunctor(_e){}
			MotionLinear(Position _origin,Position _dest,EasingFunctor *_e):MotionFunctor(_origin,_dest,_e){
				diff=dest-origin;
			}
			MotionLinear(const MotionLinear& obj){
				origin=obj.origin; dest=obj.dest; diff=dest-origin; e=obj.e;
			}
			~MotionLinear(){}
			Position func(float progress) {
				float t=e->func(progress);
				return origin+(diff*t);
			}
			Position vfunc(float progress) {
				float vt=e->vfunc(progress);
				return diff*vt;
			}
			void initialise(){
				diff=dest-origin;
				e->initialise();
			}
			float distance() const{
				Position _diff(dest-origin);
				return MyMath::sqrt(_diff.x*_diff.x+_diff.y*_diff.y);
			}
	};

	class MotionTurn : public MotionFunctor{
		private:
			Position diff;
		public:
			MotionTurn(EasingFunctor *_e):MotionFunctor(_e){}
			MotionTurn(Position _origin,Position _dest,EasingFunctor *_e):MotionFunctor(_origin,_dest,_e){
				diff=dest-origin;
			}
			MotionTurn(const MotionTurn& obj){
				origin=obj.origin; dest=obj.dest; diff=dest-origin; e=obj.e;
			}
			~MotionTurn(){}
			Position func(float progress) {
				float t=e->func(progress);
				return origin+(diff*t);
			}
			Position vfunc(float progress) {
				float vt=e->vfunc(progress);
				return diff*vt;
			}
			void initialise(){
				diff=dest-origin;
				e->initialise();
			}
			float distance() const{
				Position _diff(dest-origin);
				return MyMath::fabs(_diff.angle);
			}
	};

	class MotionArc : public MotionFunctor{
		private:
			float r;
			float ox,oy;
		public:
			MotionArc(EasingFunctor *_e):MotionFunctor(_e){}
			MotionArc(Position _origin,Position _dest,EasingFunctor *_e):MotionFunctor(_origin,_dest,_e){
				initialise();
			}
			MotionArc(const MotionArc& obj){
				origin=obj.origin; dest=obj.dest; e=obj.e;
			}
			~MotionArc(){}
			Position func(float progress) {
				// time factor calculated with an easing functor
				float t=e->func(progress);
				float alpha=(dest.angle-origin.angle)*t+origin.angle;
				return Position(ox-MyMath::cos(alpha)*r,oy-MyMath::sin(alpha)*r,alpha);
			}
			Position vfunc(float progress) {
				// time factor calculated with an easing functor
				float t=e->func(progress);
				float alpha=(dest.angle-origin.angle)*t+origin.angle;
				float vt=e->vfunc(progress);
				float valpha=(dest.angle-origin.angle)*vt;
				return Position(valpha*MyMath::sin(alpha)*r,-valpha*MyMath::cos(alpha)*r,valpha);
			}
			void initialise(){
				if(MyMath::sin(origin.angle)-MyMath::sin(dest.angle)<0.001f){
					r=MyMath::fabs((origin.x-dest.x)/(MyMath::cos(dest.angle)-MyMath::cos(origin.angle)));
				} else {
					r=MyMath::fabs((origin.y-dest.y)/(MyMath::sin(dest.angle)-MyMath::sin(origin.angle)));
				}

				float adiff=dest.angle-origin.angle;
				if(adiff>MyMath::PI) adiff-=2.f*MyMath::PI;
				if(adiff<-MyMath::PI) adiff+=2.f*MyMath::PI;

				if(adiff>0) r=-r;

				ox=dest.x+MyMath::cos(dest.angle)*r;
				oy=dest.y+MyMath::sin(dest.angle)*r;
				e->initialise();
			}
			float distance() const{
				return MyMath::fabs(r)*MyMath::fabs(dest.angle-origin.angle);
			}
	};

	class MotionSmoothArc : public MotionFunctor {
		private:
			float adiff;
			float r;
			float v;
			float x;
			float y;
			float progress_p;
		public:
			MotionSmoothArc(EasingFunctor *_e):MotionFunctor(_e),adiff(0),r(0),v(0),x(0),y(0),progress_p(0){}
			MotionSmoothArc(Position _origin,Position _dest,EasingFunctor *_e):MotionFunctor(_origin,_dest,_e),adiff(0),r(0),v(0),x(_origin.x),y(_origin.y),progress_p(0){
				initialise();
			}
			MotionSmoothArc(const MotionSmoothArc& obj){
				origin=obj.origin; dest=obj.dest; adiff=obj.adiff; r=obj.r; v=obj.r; x=origin.x; y=origin.y; progress_p=0;
			}
			~MotionSmoothArc(){}
			Position func(float progress){
				float angle;
				if(progress < 1.f/3.f) {
					angle = origin.angle + 2.25f*adiff*progress*progress;
				} else if (progress < 2.f/3.f) {
					angle = origin.angle + adiff*(1.5f*progress-0.25f);
				} else {
					angle = origin.angle + adiff*(-2.25f*progress*progress+4.5f*progress-1.25f);
				}
				x -= r*MyMath::sin(angle)*(progress-progress_p);
				y += r*MyMath::cos(angle)*(progress-progress_p);
				progress_p = progress;
				return Position(x, y, angle);
			}
			Position vfunc(float progress){
				float angle, omega;
				if(progress < 1.f/3.f) {
					angle = origin.angle + 2.25f*adiff*progress*progress;
					omega = 4.5f*adiff*progress;
				} else if (progress < 2.f/3.f) {
					angle = origin.angle + adiff*(1.5f*progress-0.25f);
					omega = 1.5f*adiff;
				} else {
					angle = origin.angle + adiff*(-2.25f*progress*progress+4.5f*progress-1.25f);
					omega = (1.f-progress)*4.5f*adiff;
				}
				float vx = -v * MyMath::sin(angle);
				float vy = v * MyMath::cos(angle);
				return Position(vx*time, vy*time, omega);
			}
			static Position calcDest(Position _origin, float _distance, float _angle, int8_t dir=-1){
				float _adiff=_angle-_origin.angle;
				if(_adiff < -MyMath::PI) _adiff += 2.f*MyMath::PI;
				if(_adiff > MyMath::PI) _adiff -= 2.f*MyMath::PI;

				// compensation for 180 deg turn
				if(dir==2) _adiff = -MyMath::fabs(_adiff); // turn right
				if(dir==8) _adiff = MyMath::fabs(_adiff); // turn left

				float xm = -0.4052f*MyMath::sin(0.4696f*_adiff) - 0.3316f*MyMath::sin(0.9392f*_adiff);
				float ym = 0.299f + 0.3518f*MyMath::cos(0.4651f*_adiff) + 0.3492f*MyMath::cos(0.9302f*_adiff);
				float origDistance = MyMath::sqrt(xm*xm+ym*ym);

				float xd = MyMath::cos(_origin.angle)*xm - MyMath::sin(_origin.angle)*ym;
				float yd = MyMath::sin(_origin.angle)*xm + MyMath::cos(_origin.angle)*ym;

				float _r = _distance/origDistance;

				//debug<<"_adiff:"<<(int32_t)(_adiff*1000)<<endl;
				return _origin + Position(xd*_r, yd*_r, _adiff);
			}

			void initialise() {
				Position diff = dest-origin;
				float distance = MyMath::sqrt(diff.x*diff.x+diff.y*diff.y);

				adiff = diff.angle;
				if(adiff < -MyMath::PI) adiff += 2.f*MyMath::PI;
				if(adiff > MyMath::PI) adiff -= 2.f*MyMath::PI;
				//debug<<"adiff:"<<(int32_t)(adiff*1000)<<endl;

				float xm = -0.4052f*MyMath::sin(0.4696f*adiff) - 0.3316f*MyMath::sin(0.9392f*adiff);
				float ym = 0.299f + 0.3518f*MyMath::cos(0.4651f*adiff) + 0.3492f*MyMath::cos(0.9302f*adiff);
				float origDistance = MyMath::sqrt(xm*xm+ym*ym);

				r = distance/origDistance;
				x = origin.x; y = origin.y;
				progress_p=0;
			}

			void configureParams(float v0,float vf,float vmax,float amax){
				time = r/v0;
				v = v0;
			}
			float distance() const{
				return r;
			}
	};

}
