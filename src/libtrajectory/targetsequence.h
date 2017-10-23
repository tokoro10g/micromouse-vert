#pragma once

#include "target.h"
#include "position.h"
#include "parameters.h"
#include "../utils/debug.h"
#include <queue>

namespace Trajectory{
	class TargetSequence{
		public:
			TargetSequence(float _dt):lastPosition(0,0,0),timestamp(0),dt(_dt){}
			~TargetSequence(){}

			void push(const Target &t){
				targetQueue.push(t);
			}

			void pushDifference(Position diff, MotionFunctor *mf, const Parameters p){
				Position dest=lastPosition+diff;
				push(dest, mf, p);
			}

			void push(Position dest, MotionFunctor *mf, const Parameters p){
				Position diff=dest-lastPosition;
				if(MyMath::fabs(diff.x)<2.f && 
						MyMath::fabs(diff.y)<2.f && 
						MyMath::fabs(MyMath::normalized(diff.angle, MyMath::PI))<0.2f){
					return;
				}
				float adiff = diff.angle;
				if(adiff>MyMath::PI){
					adiff -= 2.f*MyMath::PI;
				} else if(adiff<-MyMath::PI){
					adiff += 2.f*MyMath::PI;
				}
				if(lastPosition.angle>MyMath::PI){
					lastPosition.angle-=2.f*MyMath::PI;
				} else if(lastPosition.angle<-MyMath::PI){
					lastPosition.angle+=2.f*MyMath::PI;
				}
				if(MyMath::fabs(MyMath::fabs(adiff)-MyMath::PI)<0.01f){
					dest.angle = lastPosition.angle+adiff;
					if(dest.angle>MyMath::PI){
						lastPosition.angle -= 2.f*MyMath::PI;
						dest.angle -= 2.f*MyMath::PI;
					} else if(dest.angle<-MyMath::PI){
						lastPosition.angle += 2.f*MyMath::PI;
						dest.angle += 2.f*MyMath::PI;
					}
				} else {
					if(dest.angle>MyMath::PI){
						dest.angle-=2.f*MyMath::PI;
					} else if(dest.angle<-MyMath::PI){
						dest.angle+=2.f*MyMath::PI;
					}
					if(lastPosition.angle-dest.angle>MyMath::PI){
						dest.angle+=2.f*MyMath::PI;
					} else if(lastPosition.angle-dest.angle<-MyMath::PI){
						dest.angle-=2.f*MyMath::PI;
					}
				}
				mf->setPosition(lastPosition, dest);
				mf->initialise();
				mf->configureParams(p.v0,p.vf,p.vmax,p.amax);
				targetQueue.push(Target(mf,dt));
				lastPosition.setVars(dest.x, dest.y, dest.angle);
			}

			bool isEmpty() const{
				return targetQueue.empty();
			}

			uint16_t getTimestamp() const{
				return timestamp;
			}

			inline void incrementTimestamp() {
				if(!targetQueue.empty()) timestamp++;
			}

			Position getCurrentPosition(){
				if(!targetQueue.empty()) {
					Position p=targetQueue.front().getCurrentPosition(timestamp);
					return p;
				}
				return lastPosition;
			}

			Position getCurrentVelocity(){
				if(!targetQueue.empty()) {
					Position p=targetQueue.front().getCurrentVelocity(timestamp);
					lastVelocity = p;
					return p;
				}
				return lastVelocity;
			}

			void nextTarget(){
				if(!targetQueue.empty() && timestamp>=(targetQueue.front().getTime())){
					timestamp=0;
					targetQueue.front().destroy();
					targetQueue.pop();
					//dleds[2].set();
				}
			}

			Position getLastPosition() const{
				return lastPosition;
			}

			void resetSequence(const Position& pos){
				lastPosition=pos;
				while(!targetQueue.empty()){
					targetQueue.front().destroy();
					targetQueue.pop();
				}
			}

		private:
			std::queue<Target> targetQueue;
			Position lastPosition;
			Position lastVelocity;
			uint16_t timestamp;
			float dt;
	};
}
