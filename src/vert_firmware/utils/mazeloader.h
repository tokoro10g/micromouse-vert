#include "maze.h"

class MazeLoader {
	private:
		MazeLoader();
	public:
		static void loadEmpty(int w,int h,int x,int y,MazeSolver::Maze &maze){
			maze.resize(w,h);
			maze.setType(1);
			maze.setGoal(x,y);
			for(int i=0;i<h;i++){
				for(int j=0;j<w;j++){
					MazeSolver::CellData cell={0};
					if(j==0){
						cell.bits.WEST=1;
					} else if(j==w-1){
						cell.bits.EAST=1;
					}
					if(i==0){
						cell.bits.SOUTH=1;
					} else if(i==h-1){
						cell.bits.NORTH=1;
					}
					if(i==0&&j==0){
						cell.bits.EAST=1;
						cell.bits.CHK_EAST=1;
					}
					if(i==0&&j==1){
						cell.bits.WEST=1;
						cell.bits.CHK_WEST=1;
					}
					maze.setCell(i*w+j,cell);
				}
			}
		}
		static void loadTestMaze(int w,int h,int x,int y,MazeSolver::Maze &maze){
			maze.resize(w,h);
			maze.setType(1);
			maze.setGoal(x,y);
			//static const char *data="9551515395151553a9543c3aa969453aaa954382869453aa82a9542aa9453a82a82a952aa853a86a82c2a92aac3a8692ac3c46ec6944696ac545553916955456951513ac6bc55153a9692aa916913c3a8692aaaaa96a83c2a92ac42a8056a83a82ac512aac156a82aac5168283c156aaac51696aac3c156aed545456c5454556";
			//static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad2aac3ffffffffaa96c416ffffffffec6d3babffffffff9393842affffffffac2ac16affffffffa92a9416ffffffffeec6c56fffffffff";
			//static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad2aac3ffffffffaa96c416ffffffffec6d3babffffffffd513842affffffff956ac16affffffffa9129416ffffffffeeeec56fffffffff";
			//static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad2aac3ffffffffaa96c416ffffffffec6d3babffffffff9393842affffffffac6ac16affffffffa93a9416ffffffffeec6c56fffffffff";
			//static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad2aac3ffffffffaa96c416ffffffffec6d3babffffffff9553842affffffffa93ac16affffffffaa869416ffffffffeec7c56fffffffff";
			//static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad2aac3ffffffffaa96c416ffffffffec6d3babffffffff93d3842affffffffac3ac16affffffffabc29416ffffffffec56c56fffffffff";


//static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad2aac3ffffffffaa96c416ffffffffec6d3babffffffffd113842affffffff96aac16affffffffabea9416ffffffffec56c56fffffffff";

//static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad2aac3ffffffffaa96c416ffffffffec6d3babffffffff9113842affffffffaeaac16affffffffa96a9416ffffffffec56c56fffffffff";


//static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad2aac3ffffffffaa96c416ffffffffec6d7fabffffffff939393aaffffffffac6c6aeaffffffffabd15296ffffffffec56d6efffffffff";
static const char *data = "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff93fffffff93d1113c6fffffffaad6aac3ffffffffaa93c416ffffffffaaaaffabffffffffac6a93aaffffffffa93aeaeaffffffffaaaad296ffffffffeec6d6efffffffff";


			for(int i=0;i<16;i++){
				for(int j=0;j<16;j++){
					MazeSolver::CellData cell={0};
					char c=data[16*i+j];
					if(c>='0'&&c<='9'){
						c-='0';
					} else if(c>='a'&&c<='f'){
						c-='a';
						c+=10;
					}
					cell.half=c;
					maze.setCell((15-i)*w+j,cell);
				}
			}
		}
};
