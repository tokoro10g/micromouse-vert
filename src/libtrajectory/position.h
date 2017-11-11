#pragma once

namespace Trajectory {
	class Position {
		private:
		public:
			float x;
			float y;
			float angle;
			Position():x(0),y(0),angle(0){}
			Position(float _x,float _y,float _angle):x(_x),y(_y),angle(_angle){}
			Position(const Position &p){
				x=p.x; y=p.y; angle=p.angle;
			}

			void setVars(float _x,float _y,float _angle){ x=_x; y=_y; angle=_angle; }
			const Position operator- (const Position& rhs) const { return Position(x-rhs.x,y-rhs.y,angle-rhs.angle); }
			const Position operator+ (const Position& rhs) const { return Position(x+rhs.x,y+rhs.y,angle+rhs.angle); }
			bool operator== (const Position& rhs) const { return (x==rhs.x)&&(y==rhs.y)&&(angle==rhs.angle); }
			template <typename T> Position operator/ (const T d) const{ return Position((T)x/d,(T)y/d,(T)angle/d); }
			template <typename T> Position operator* (const T d) const{ return Position((T)x*d,(T)y*d,(T)angle*d); }
			template <typename T> void operator*= (const T d) { x*=d; y*=d; angle*=d; }
			const Position rot(float angle) const;
	};
	const Position operator* (const float c, const Position pos);
	const Position operator* (const Position pos, const float c);
	const Position operator/ (const Position pos, const float c);
}
