#include "position.h"
#include "mymath.h"

namespace Trajectory {
	const Position operator* (const float c, const Position pos) { return Position(pos.x*c,pos.y*c,pos.angle*c); }
	const Position operator* (const Position pos, const float c) { return Position(pos.x*c,pos.y*c,pos.angle*c); }
	const Position operator/ (const Position pos, const float c) { return Position(pos.x/c,pos.y/c,pos.angle/c); }
	const Position Position::rot(float a) const {
		float sina = MyMath::sin(a);
		float cosa = MyMath::cos(a);
		return Position(cosa*x-sina*y, sina*x+cosa*y, a+angle);
	}
}
