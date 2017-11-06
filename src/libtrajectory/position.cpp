#include "position.h"

namespace Trajectory {
	const Position operator* (const float c, const Position pos) { return Position(pos.x*c,pos.y*c,pos.angle*c); }
	const Position operator* (const Position pos, const float c) { return Position(pos.x*c,pos.y*c,pos.angle*c); }
	const Position operator/ (const Position pos, const float c) { return Position(pos.x/c,pos.y/c,pos.angle/c); }
}
