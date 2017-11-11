#include "maze.h"
#include "wallsource.h"
#include "../machine/machine.h"

namespace MazeSolver {
	class MachineWallSource : public WallSource {
		public:
			MachineWallSource(Vert::Machine *_machine):WallSource(),machine(_machine){}
			~MachineWallSource(){}
			bool isSetWall(Coord c, Direction d, Direction rd){
				return machine->isSetWall(rd.half);
			}
		private:
			Vert::Machine *machine;
	};
}
