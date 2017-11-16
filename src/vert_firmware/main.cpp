#include "main.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

#include "machine/machine.h"
#include "mymath.h"

#include "control/parameters.h"

#include "string.h"

#include "../libmazesolver/maze.h"
#include "../libmazesolver/agent.h"
#include "utils/mazeloader.h"
#include "utils/machinewallsource.h"

#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

using namespace Vert;
using namespace MazeSolver;

UART_HandleTypeDef huart1;

int main(void) __attribute__((optimize("O0")));

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

void test_IMUs();
void test_encoders();

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

Machine machine;

int16_t turnCount;
bool isEndOfSequence;

Maze maze;
Maze backupMaze;
Graph graph;
MachineWallSource wallSource(&machine);
Agent agent(&maze,&wallSource,&graph);

uint16_t g_index = 31;
Direction g_dir = Maze::DirNorth;
uint16_t g_goal = 31;
float g_angle = 0;

float CellRatio = 0.5f;

void playMario(){
	buzzer.stop();
	buzzer.addNote(0, 20);
	buzzer.addNote('e', 6, 100);
	buzzer.addNote(0, 50);
	buzzer.addNote('e', 6, 100);
	buzzer.addNote(0, 200);
	buzzer.addNote('e', 6, 100);
	buzzer.addNote(0, 200);
	buzzer.addNote('c', 6, 150);
	buzzer.addNote('e', 6, 100);
	buzzer.addNote(0, 200);
	buzzer.addNote('g', 6, 150);
	buzzer.addNote(0, 450);
	buzzer.addNote('g', 5, 150);
	buzzer.play();
}
void playBootSound(){
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote('e', 7, 100);
	buzzer.addNote(0, 50);
	buzzer.addNote('d', 7, 100);
	buzzer.addNote(0, 50);
	buzzer.addNote('c', 7, 100);
	buzzer.addNote(0, 200);
	buzzer.addNote('g', 6, 150);
	buzzer.addNote(0, 150);
	buzzer.addNote('g', 7, 150);
	buzzer.addNote(0, 150);
	buzzer.addNote('e', 7, 100);
	buzzer.play();
}
void playStartSound2(){
	if(MyMath::Machine::CellWidth!=90.f){ playMario(); return; }
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote('c', 5, 100);
	buzzer.addNote('g', 5, 100);
	buzzer.addNote('c', 6, 100);
	buzzer.addNote('d', 6, 100);
	buzzer.addNote('g', 6, 100);
	buzzer.addNote('c', 7, 100);
	buzzer.addNote('d', 7, 100);
	buzzer.addNote('c', 7, 100);
	buzzer.addNote('g', 6, 100);
	buzzer.addNote('d', 6, 100);
	buzzer.addNote('c', 6, 100);
	buzzer.addNote('g', 5, 100);
	buzzer.addNote('g', 5, 50);
	buzzer.addNote('c', 6, 50);
	buzzer.addNote('e', 6, 300);
	buzzer.play();
}
void playStartSound(){
	if(MyMath::Machine::CellWidth!=90.f){ playMario(); return; }
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote('c', 7, 20);
	buzzer.addNote('e', 7, 20);
	buzzer.addNote('f', 7, 140);
	buzzer.addNote('e', 7, 180);
	buzzer.addNote('c', 7, 180);
	buzzer.addNote('g', 6, 180);
	buzzer.addNote('g', 6, 20);
	buzzer.addNote('c', 7, 20);
	buzzer.addNote('d', 7, 140);
	buzzer.addNote('c', 7, 220);
	buzzer.addNote(7459/4, 260);
	buzzer.addNote('f', 6, 300);
	buzzer.addNote(7459/4, 40);
	buzzer.addNote(4978/2, 40);
	buzzer.addNote('g', 7, 300);
	buzzer.play();
}
void playErrorSound(){
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote(2093, 130);
	buzzer.addNote(1046, 130);
	buzzer.addNote(2093, 130);
	buzzer.addNote(1046, 130);
	buzzer.play();
}
void playConfirmSound(){
	buzzer.stop();
	buzzer.addNote(0,1);
	buzzer.addNote(2093, 80);
	buzzer.addNote(1568, 80);
	buzzer.play();
}
void playConfirmSoundUp(){
	buzzer.stop();
	buzzer.addNote(0,1);
	buzzer.addNote(1568, 80);
	buzzer.addNote(2093, 80);
	buzzer.play();
}
void playEndSound(){
	buzzer.stop();
	buzzer.addNote(0, 2);
	buzzer.addNote('b', 5, 250);
	buzzer.addNote(0, 300);
	buzzer.addNote('c', 6, 250);
	buzzer.addNote(0, 300);
	buzzer.addNote('c', 7, 150);
	buzzer.play();
}

uint8_t modeSelect(uint8_t max){
	int8_t val=0;
	int8_t changed=0;
	encoderR.reset();
	while(1){
		int16_t enc=encoderR.getCounter();
		if(enc<-500){
			val++; encoderR.reset(); changed=1;
		} else if(enc>500){
			val--; encoderR.reset(); changed=-1;
		}

		if(val<0) val=max; else if(val>max) val=0;

		if(changed!=0){
			/*
			// korikori
			motors.setOutput(0,changed*20);
			HAL_Delay(20);
			motors.setOutput(0, 0);
			*/
			if(max<8){
				for(uint8_t i=0;i<6;i+=2){ leds.write(i,val&(1<<(i/2))); leds.write(i+1,val&(1<<(i/2))); }
			} else {
				for(uint8_t i=0;i<6;i++){ leds.write(i,val&(1<<i)); }
			}
		}

		button.updateState();
		if(button.getState()==Button::RisingEdge){
			for(uint8_t i=0;i<6;i++) { leds.reset(i); }
			playConfirmSoundUp();
			return val;
		}

		HAL_Delay(20); changed=0;
	}
}

int8_t waitIR(){
	machine.unblock();
	for (uint8_t i = 0; i < 6; ++i) { leds.reset(i); }
	HAL_Delay(200);
	uint8_t cnt = 0;
	while(machine.getADCValue(IRSensor::RB)<2000){
		cnt++;
		button.updateState();
		if(button.getState()==Button::RisingEdge){
			playConfirmSound();
			return -1;
		}
		HAL_Delay(20);
		if(cnt%8==0){ for (uint8_t i = 0; i < 6; ++i) { leds.toggle(i); } }
	}
	for (uint8_t i = 0; i < 6; ++i) { leds.reset(i); }
	HAL_Delay(20);
	machine.block();
	return 0;
}

void initialiseRun(){
	graph.loadMaze(&maze);

	agent.setIndex(maze.getWidth()-1);
	agent.setDir(Maze::DirNorth);
	agent.reroute();

	g_index=maze.getWidth()-1;
	g_angle=0;
	g_dir=Maze::DirNorth;

	turnCount = 0;

	machine.deactivate();
	machine.refreshIMUOffsets();
	machine.unblock();

	encoderL.reset();
	encoderR.reset();
	machine.setState(0,MyMath::Machine::InitialY,0);
	machine.resetTargetSequence(Trajectory::Position(0,MyMath::Machine::InitialY,0));
	machine.resetControllers();
	//machine.setWallAdjust();
	machine.setNeutralSideSensorValue();
	machine.activate();
	isEndOfSequence = false;
}

void pullBack(){
	using namespace MyMath;
	using namespace MyMath::Machine;
	using namespace Trajectory;

	if(machine.getLastVelocity()!=0){
		const float d = CellWidth/2.f-(PreTurnDistance+PreSensDistance);
		machine.pushTargetDiff(Position(-d*sin(g_angle),d*cos(g_angle),0.f), new MotionLinear(new EasingTrap()), p_straight_end);
		//machine.setFrontWallCorrection(true);
		HAL_Delay(600);
		//machine.disableLog();
		//machine.setWallCorrection(false);
		//flushLog();
	} else {
		HAL_Delay(1000);
	}
	//machine.setWallCorrection(false);
	//machine.setFrontWallCorrection(false);
	//flushLog();

	if(turnCount>0) {
		machine.pushTargetDiff(Position(0,0,-PI), new MotionTurn(new EasingPoly5()), p_miniturn);
		turnCount-=2;
	} else {
		machine.pushTargetDiff(Position(0,0,PI), new MotionTurn(new EasingPoly5()), p_miniturn);
		turnCount+=2;
	}
	HAL_Delay(600);
	//machine.resetAdjustment();
	//machine.setWallCorrection(true);
	HAL_Delay(200);
	//flushLog();
	machine.pushTargetDiff(Position(-(CellWidth/2.f+PreSensDistance)*sin(g_angle+PI),(CellWidth/2.f+PreSensDistance)*cos(g_angle+PI),0.f), new MotionLinear(new EasingTrap()), p_straight_start);

	g_angle=normalized(g_angle+PI,PI);
	g_dir=agent.getDir();
	//machine.enableLog();
	return;
}
int8_t procSearch(uint16_t _goal, bool further=false){
	using namespace MyMath;
	using namespace MyMath::Machine;
	using namespace Trajectory;
	g_goal=_goal;

	uint8_t pullBackCount = 0;
	Direction straightDir;
	uint8_t straightCount = 0;
	while(1){
		if(machine.isTargetSequenceEmpty()){
			int16_t nextIndex;
			if(!isEndOfSequence && straightCount==0){
				machine.pushTargetDiff(Position(-PreTurnDistance*sin(g_angle),PreTurnDistance*cos(g_angle),0), new MotionLinear(new EasingLinear()), p_turn);
			}

			int16_t edges[6]={};
			bool isVisitedEdges[6]={};
			bool isObservedEdges[6]={};
			uint8_t edgescnt = graph.getEdges(g_index, edges);
			for(uint8_t i=0; i<edgescnt; i++){
				Node n = graph.getNode(edges[i]);
				isObservedEdges[i] = maze.isSearchedCell(n.getCoord(maze.getWidth()));
				isVisitedEdges[i] = graph.getNode(edges[i]).isVisited();
			}

			//dleds[5].set();
			nextIndex=agent.stepMaze(g_goal, true);
			//dleds[5].reset();

			bool isObserved = false;
			bool isVisited = false;
			for(uint8_t i=0; i<edgescnt; i++){
				if(edges[i]==nextIndex){
					isObserved = isObservedEdges[i];
					isVisited = isVisitedEdges[i];
					break;
				}
			}

			//debug<<nextIndex<<endl;
			if(nextIndex<0){
				if(!further){
					playErrorSound();
					machine.deactivate();
				}
				return -2;
			}
			if(nextIndex==g_index||agent.isPullBack(g_index,nextIndex,g_dir,agent.getDir())){
				if(straightCount>0){
					float a=-(float)(straightDir.half==0x2)*PI/2.f-(float)(straightDir.half==0x4)*PI+(float)(straightDir.half==0x8)*PI/2.f;
					if(straightCount==1){
						machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingLinear()), p_straight);
					} else {
						machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingTrap()), p_straight_acc);
					}
					straightCount=0;
				}
				pullBack();
				//flushLog();
				if(pullBackCount==0) {
					pullBackCount++;
					if(further){
						return -2;
					}
					continue;
				} else {
					playErrorSound();
					machine.deactivate();
					return -3;
				}
			} else {
				pullBackCount = 0;
			}
			//flushLog();
			g_dir=agent.getDir();
			Coord c=agent.getCoord();
			Position p(c.x*CellWidth,c.y*CellWidth,0);
			p.angle=-(float)(g_dir.half==0x2)*PI/2.f-(float)(g_dir.half==0x4)*PI+(float)(g_dir.half==0x8)*PI/2.f;
			p.x+=(g_dir.half==0x2)*(CellWidth/2.f+PreSensDistance)-(g_dir.half==0x8)*(CellWidth/2.f+PreSensDistance);
			p.y+=(g_dir.half==0x1)*(CellWidth/2.f+PreSensDistance)-(g_dir.half==0x4)*(CellWidth/2.f+PreSensDistance);
#ifndef MAZEDEBUG
			if(fabs(normalized(g_angle-p.angle,PI))<0.2f){
				if(isObserved){
					straightCount++;
					straightDir.half = g_dir.half;
					if(!isVisited){
						float a=-(float)(straightDir.half==0x2)*PI/2.f-(float)(straightDir.half==0x4)*PI+(float)(straightDir.half==0x8)*PI/2.f;
						if(straightCount==1){
							machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingLinear()), p_straight);
						} else {
							machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingTrap()), p_straight_acc);
						}
						straightCount=0;
					}
				} else {
					if(straightCount>0){
						float a=-(float)(straightDir.half==0x2)*PI/2.f-(float)(straightDir.half==0x4)*PI+(float)(straightDir.half==0x8)*PI/2.f;
						if(straightCount==1){
							machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingLinear()), p_straight);
						} else {
							machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingTrap()), p_straight_acc);
						}
						straightCount=0;
					}
					machine.pushTarget(p, new MotionLinear(new EasingTrap()), p_straight);
				}
			} else {
				if(straightCount>0){
					float a=-(float)(straightDir.half==0x2)*PI/2.f-(float)(straightDir.half==0x4)*PI+(float)(straightDir.half==0x8)*PI/2.f;
					if(straightCount==1){
						machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingLinear()), p_straight);
					} else {
						machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingTrap()), p_straight_acc);
					}
					straightCount=0;
				}
				Position pp = machine.getLastPosition();
				if(signof(normalized(p.angle-g_angle, PI) > 0)){
					turnCount++;
				} else {
					turnCount--;
				}
				machine.pushTarget(p-Position(-(PreTurnDistance+2.f*PreSensDistance)*sin(p.angle),(PreTurnDistance+2.f*PreSensDistance)*cos(p.angle),0), new MotionSmoothArc(new EasingLinear()), p_turn);
				machine.pushTargetDiff(Position(-(PreTurnDistance+2.f*PreSensDistance)*sin(p.angle),(PreTurnDistance+2.f*PreSensDistance)*cos(p.angle),0), new MotionLinear(new EasingLinear()), p_turn);
				//machine.pushTarget(p, new MotionSmoothArc(new EasingLinear()), p_turn);
			}
#endif
			g_angle=normalized(p.angle,PI);
			g_index=nextIndex;
			if(nextIndex==g_goal){
				if(straightCount>0){
					float a=-(float)(straightDir.half==0x2)*PI/2.f-(float)(straightDir.half==0x4)*PI+(float)(straightDir.half==0x8)*PI/2.f;
					if(straightCount==1){
						machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingLinear()), p_straight);
					} else {
						machine.pushTargetDiff(Position(-CellWidth*sin(a)*straightCount,CellWidth*cos(a)*straightCount,0), new MotionLinear(new EasingTrap()), p_straight_acc);
					}
					straightCount=0;
				}
				backupMaze.replace(maze);
				return 0;
			}
			//flushLog();
		}

		//flushLog();
		if(!machine.isActivated()) break;

		HAL_Delay(1);
	}
	//flushLog();

	playErrorSound();
	return -1;
}

int8_t searchRunMode(bool infinityMode=false){
	using namespace MyMath;
	using namespace MyMath::Machine;
	using namespace Trajectory;

	uint8_t param=modeSelect(4);
	if(param==0) return 0;

	int16_t v = 75+param*(50+(CellWidth==180.f)*50);
	p_straight_start=Trajectory::Parameters(0,v,v,2000);
	p_straight=Trajectory::Parameters(v,v,v,2000);
	p_straight_acc=Trajectory::Parameters(v,v,v+200,700+param*100);
	p_straight_end=Trajectory::Parameters(v,0,v,1500);
	p_turn=Trajectory::Parameters(v,v,v,2000);
	p_ministraight=Trajectory::Parameters(0,0,v,1000);

	if(waitIR()<0) return 0;
	playStartSound2();
	HAL_Delay(3000);

	initialiseRun();
	/*
	machine.setWallEdgeCorrection(false);
	machine.setWallCorrection(true);
	*/

	do {
		machine.pushTarget(Position(0,CellWidth/2.f+PreSensDistance,0), new MotionLinear(new EasingTrap()), p_straight_start);

		if(procSearch(maze.getGoalNodeIndex())==0) {
			if(infinityMode){
				MazeLoader::loadEmpty(32,32,maze.getGoalX(),maze.getGoalY(),maze);
				graph.loadEmpty(32,32);
				graph.loadMaze(&maze);
				graph.reset();
			}
			playConfirmSound();
		} else {
			//flushLog();
			maze.replace(backupMaze);
			return -1;
		}

		agent.reroute();

		while(1){
			int16_t tempGoal=agent.searchCandidateNode(maze.getWidth()-1);
			if(tempGoal<0){
				backupMaze.replace(maze);
				playEndSound();
				break;
			}
			agent.reroute();
			int8_t result=procSearch(tempGoal,true);
			if(result>=0){
				graph.getNodePointer(tempGoal)->setVisited();
			} else {
				if(tempGoal!=maze.getWidth()-1){
					continue;
				}
				//FIXME: enters here for a specific error condition
				playErrorSound();
				maze.replace(backupMaze);
				machine.deactivate();
				machine.block();
				//flushLog();
				return -1;
			}
		}

		agent.reroute();
		if(procSearch(maze.getWidth()-1)!=0) {
			//flushLog();
			playErrorSound();
			maze.replace(backupMaze);
			machine.deactivate();
			machine.block();
			return -1;
		}

		machine.pushTargetDiff(Position(-CellWidth/2.f*sin(g_angle),CellWidth/2.f*cos(g_angle),0.f), new MotionLinear(new EasingTrap()), p_straight_end);
		HAL_Delay(3000);
		playConfirmSound();
		if(infinityMode){
			MazeLoader::loadEmpty(32,32,maze.getGoalX(),maze.getGoalY(),maze);
			graph.loadEmpty(32,32);
			graph.loadMaze(&maze);
			graph.reset();
			machine.pushTargetDiff(Position(0.f,0.f,PI), new MotionTurn(new EasingPoly5()), p_miniturn);
			g_index=maze.getWidth()-1;
			g_angle=0;
			g_dir=Maze::DirNorth;
			turnCount = 0;
			isEndOfSequence = false;
			agent.setIndex(maze.getWidth()-1);
			agent.setDir(Maze::DirNorth);
			agent.reroute();
		}
	} while(infinityMode);

	machine.deactivate();
	//flushLog();
	machine.block();
	return 0;
}

bool pushDiagonalTrajectory(int16_t alphaFrom, int16_t alphaTo, int8_t dir=-1) {
	using namespace Trajectory;
	using namespace MyMath;
	int16_t mod = (alphaTo - alphaFrom) % 360;
	if((alphaTo - alphaFrom) % 360 == 0) return false;

	bool ret = true;

	float alphaFrom_rad = (float)alphaFrom / 180.f * MyMath::PI;
	float alphaTo_rad = (float)alphaTo / 180.f * MyMath::PI;

	switch(abs(mod)) {
		case 180:
			{
				if(dir<=0) break;
				// 180 deg turn
				//machine.pushTargetDiff(20.f*CellRatio*Position(-sin(alphaFrom_rad), cos(alphaFrom_rad), 0.f), new MotionLinear(new EasingLinear()), p_fastturn);
				Position po = MotionSmoothArc::calcDest(machine.getLastPosition(), Machine::CellWidth, alphaTo_rad, dir);
				machine.pushTarget(po, new MotionSmoothArc(new EasingLinear()), p_fastturn);
				//machine.pushTargetDiff(20.f*CellRatio*Position(-sin(alphaTo_rad), cos(alphaTo_rad), 0.f), new MotionLinear(new EasingLinear()), p_fastturn);
			}
			break;
		case 45:
		case 315:
			{
				// 45 deg turn
				if(alphaFrom % 90 != 0) {
					machine.pushTargetDiff(52.1353f*CellRatio*Position(-sin(alphaFrom_rad), cos(alphaFrom_rad), 0.f), new MotionLinear(new EasingLinear()), p_fastturn);
				}
				Position po = MotionSmoothArc::calcDest(machine.getLastPosition(), 138.7150f*CellRatio, alphaTo_rad);
				machine.pushTarget(po, new MotionSmoothArc(new EasingLinear()), p_fastturn);
				if(alphaFrom % 90 == 0) {
					machine.pushTargetDiff(52.1353f*CellRatio*Position(-sin(alphaTo_rad), cos(alphaTo_rad), 0.f), new MotionLinear(new EasingLinear()), p_fastturn);
				}
			}
			break;
		case 90:
		case 270:
			{
				// 90 deg turn
				if(alphaFrom % 90 == 0) {
					float dist = 233.5392f*CellRatio;
					//float dist = 205.2315f*CellRatio;
					//machine.pushTargetDiff(20.f*CellRatio*Position(-sin(alphaFrom_rad), cos(alphaFrom_rad), 0.f), new MotionLinear(new EasingLinear()), p_fastturn);
					Position po = MotionSmoothArc::calcDest(machine.getLastPosition(), dist, alphaTo_rad);
					machine.pushTarget(po, new MotionSmoothArc(new EasingLinear()), p_fastturn);
					//machine.pushTargetDiff(20.f*CellRatio*Position(-sin(alphaTo_rad), cos(alphaTo_rad), 0.f), new MotionLinear(new EasingLinear()), p_fastturn);
				} else {
					/*
					machine.pushTargetDiff(7.f*CellRatio*sqrt(2.f)*Position(-sin(alphaFrom_rad),cos(alphaFrom_rad),0), new MotionLinear(new EasingLinear()), p_fastturn);
					Position po = MotionSmoothArc::calcDest(machine.getLastPosition(), 166.1380f*CellRatio, alphaTo_rad);
					machine.pushTarget(po, new MotionSmoothArc(new EasingLinear()), p_fastturn);
					machine.pushTargetDiff(7.f*CellRatio*sqrt(2.f)*Position(-sin(alphaTo_rad),cos(alphaTo_rad),0), new MotionLinear(new EasingLinear()), p_fastturn);
					*/
					machine.pushTargetDiff(10.f*CellRatio*sqrt(2.f)*Position(-sin(alphaFrom_rad),cos(alphaFrom_rad),0), new MotionLinear(new EasingLinear()), p_fastturn);
					Position po = MotionSmoothArc::calcDest(machine.getLastPosition(), 160.f*CellRatio, alphaTo_rad);
					machine.pushTarget(po, new MotionSmoothArc(new EasingLinear()), p_fastturn);
					machine.pushTargetDiff(10.f*CellRatio*sqrt(2.f)*Position(-sin(alphaTo_rad),cos(alphaTo_rad),0), new MotionLinear(new EasingLinear()), p_fastturn);
				}
			}
			break;
		case 135:
		case 225:
			{
				// 135 deg turn
				if(alphaFrom % 90 == 0) {
					machine.pushTargetDiff(10.f*CellRatio*Position(-sin(alphaFrom_rad),cos(alphaFrom_rad),0), new MotionLinear(new EasingLinear()), p_fastturn);
				} else {
					machine.pushTargetDiff(9.2824f*CellRatio*Position(-sin(alphaFrom_rad),cos(alphaFrom_rad),0), new MotionLinear(new EasingLinear()), p_fastturn);
				}
				Position po = MotionSmoothArc::calcDest(machine.getLastPosition(), 187.6207f*CellRatio, alphaTo_rad);
				machine.pushTarget(po, new MotionSmoothArc(new EasingLinear()), p_fastturn);
				if(alphaFrom % 90 == 0) {
					machine.pushTargetDiff(9.2824f*CellRatio*Position(-sin(alphaTo_rad),cos(alphaTo_rad),0), new MotionLinear(new EasingLinear()), p_fastturn);
				} else {
					machine.pushTargetDiff(10.f*CellRatio*Position(-sin(alphaTo_rad),cos(alphaTo_rad),0), new MotionLinear(new EasingLinear()), p_fastturn);
				}
			}
			break;
		default:
			ret = false;
			break;
	}
	return ret;
}

int8_t fastRunMode(bool useDiagonal, bool wallAdjust){
	using namespace MyMath;
	using namespace MyMath::Machine;
	using namespace Trajectory;

	//machine.setWallCorrection(wallAdjust);

	uint8_t param=modeSelect(7);
	if(param==0) return 0;

	p_faststraight_start = Parameters(0, 250, 250, 1500);

	p_faststraight = pa_faststraight[param-1];
	p_faststraight.v0 = 250;

	p_faststraight_end = pa_faststraight_end[param-1];
	p_faststraight_end.v0 = 250;

	p_fastturn = pa_fastturn[param-1];
	p_fastturn.v0 = 250;

	if(waitIR()<0) return 0;
	playStartSound();
	HAL_Delay(3000);

	initialiseRun();

	//machine.setWallCorrection(wallAdjust);

	g_goal=maze.getGoalNodeIndex();

	//machine.selectHighSpeedGain();

	Position pTo(0,CellWidth/2.f,0);
	Position pPivot(pTo),pFrom(pTo);
	//int16_t nextIndex;
	uint8_t straightCount=0;
	bool firstFlag=true;

	int16_t alphaStart = 0;
	int16_t alphaHalfway = 0;
	int16_t alphaHalfway_pre = 0;
	int16_t alphaEnd = 0;
	bool contSequence = false;
	bool contSequence_pre = false;

	Direction dirdiff_pre; dirdiff_pre.half = 0;

	Route route;
	agent.getShortestRoute(maze.getWidth()-1, g_goal, route);
	if(route.size()<=2){
		playErrorSound();
		machine.deactivate();
		return -1;
	}

	while(g_index!=g_goal){
		//for(auto nextIndex : route){
		//if(nextIndex==maze.getWidth()-1) continue;
		//int16_t stepIndex=agent.stepMaze(g_goal, false);
		int16_t nextIndex=agent.stepMaze(g_goal, false);
		if(nextIndex<0){
			playErrorSound();
			machine.deactivate();
			return -1;
		}

		pFrom=pPivot;
		pPivot=pTo;

		Coord c=graph.getNodePointer(nextIndex)->getCoord(maze.getWidth()); // Coordinate with direction north and east
		pTo=Position(c.x*CellWidth,c.y*CellWidth,0);
		pTo.x+=(c.dir.half==0x2)*CellWidth/2.f;
		pTo.y+=(c.dir.half==0x1)*CellWidth/2.f;
		pTo.angle=-(float)(agent.getDir().half==0x2)*PI/2.f-(float)(agent.getDir().half==0x4)*PI+(float)(agent.getDir().half==0x8)*PI/2.f;

		if(useDiagonal){
			// trajectory with diagonal path {{{
			Direction dirdiff = g_dir-agent.getDir();
			if(!dirdiff.bits.NORTH) {
				Position po = pPivot - 75.f*CellRatio * Position(-sin(pPivot.angle), cos(pPivot.angle), 0.f);
				if(firstFlag){
					machine.pushTarget(Position(0,15.f*CellRatio,0), new MotionLinear(new EasingTrap()), p_faststraight_start);
					if(straightCount>0){
						machine.pushTarget(po, new MotionLinear(new EasingTrap()), p_faststraight);
					}
					p_faststraight_start = pa_faststraight_start[param-1];
					p_faststraight = pa_faststraight[param-1];
					p_faststraight_end = pa_faststraight_end[param-1];
					p_fastturn = pa_fastturn[param-1];
					firstFlag=false;
				} else if(straightCount>0){
					machine.pushTarget(po, new MotionLinear(new EasingTrap()), p_faststraight);
				}
			}

			int16_t dalpha = (dirdiff.bits.WEST) * 90 - (dirdiff.bits.EAST) * 90;

			if((dirdiff_pre.bits.WEST && dirdiff.bits.EAST) || (dirdiff_pre.bits.EAST && dirdiff.bits.WEST)) {
				if(alphaHalfway % 90 == 0) {
					alphaHalfway += dalpha / 2;
				}
				contSequence = true;
			} else {
				alphaHalfway += dalpha;
				contSequence = false;
			}
			alphaEnd += dalpha;

			// normalize start angle
			if(alphaStart > 180) alphaStart -= 360;
			else if(alphaStart <= -180) alphaStart += 360;
			// normalize angles (here, -180 deg is included exceptionally)
			if(alphaHalfway > 180) alphaHalfway -= 360;
			else if(alphaHalfway < -180) alphaHalfway += 360;
			if(alphaEnd > 180) alphaEnd -= 360;
			else if(alphaEnd < -180) alphaEnd += 360;

			// the beginning point here is 75/2 mm behind pPivot or the end of previous turn
			if(dirdiff.bits.NORTH) {
				if(!dirdiff_pre.bits.NORTH) {
					if(contSequence_pre) {
						// make the end of an existing turn
						// next angle : alphaHalfway_pre
						if(pushDiagonalTrajectory(alphaStart, alphaHalfway_pre)) {
							float alphaHalfway_pre_rad = (float)alphaHalfway_pre/180.f * PI;
							Position po(pFrom.x, pFrom.y, alphaHalfway_pre_rad);
							machine.pushTarget(po, new MotionLinear(new EasingTrap()), p_fastturn);
							alphaStart = alphaHalfway_pre;
						}
					}
					// then, terminate the whole turn sequence
					// next angle : alphaEnd
					pushDiagonalTrajectory(alphaStart, alphaEnd, dirdiff_pre.half);
					alphaStart = alphaEnd;
					alphaHalfway = alphaEnd;
				}
				// continue counting
				straightCount++;
			} else if(contSequence_pre && !contSequence) {
				// previous turn has been terminated
				// next angle : alphaHalfway_pre
				pushDiagonalTrajectory(alphaStart, alphaHalfway_pre);

				float alphaHalfway_pre_rad = (float)alphaHalfway_pre/180.f * PI;
				Position po(pFrom.x, pFrom.y, alphaHalfway_pre_rad);
				machine.pushTarget(po, new MotionLinear(new EasingTrap()), p_fastturn);
				straightCount=0;
				alphaStart = alphaHalfway_pre;
			} else {
				// do nothing
				straightCount=0;
			}

			// end
			if(nextIndex==g_goal){
				if(contSequence) {
					// make the end of an existing turn
					// next angle : alphaHalfway
					if(pushDiagonalTrajectory(alphaStart, alphaHalfway)) {
						float alphaHalfway_rad= (float)alphaHalfway/180.f * PI;
						Position po(pPivot.x, pPivot.y, alphaHalfway_rad);
						machine.pushTarget(po, new MotionLinear(new EasingTrap()), p_fastturn);
						alphaStart = alphaHalfway;
					}
				}
				if(alphaStart != alphaEnd) {
					// terminate the whole turn sequence
					// next angle : alphaEnd
					pushDiagonalTrajectory(alphaStart, alphaEnd, dirdiff_pre.half);
				}
				if(straightCount > 0) {
					machine.pushTarget(pTo, new MotionLinear(new EasingTrap()), p_faststraight);
				}
				Position po = pTo + CellWidth/2.f * Position(-sin(pTo.angle),cos(pTo.angle),0.f);
				machine.pushTarget(po, new MotionLinear(new EasingTrap()), p_faststraight_end);
			}

			alphaHalfway_pre = alphaHalfway;
			if(alphaHalfway_pre > 180) alphaHalfway_pre -= 360;
			if(alphaHalfway_pre <= -180) alphaHalfway_pre += 360;
			contSequence_pre = contSequence;
			dirdiff_pre = dirdiff;
			// }}}
		} else {
			// trajectory without diagonal path {{{
			if(g_dir.half==agent.getDir().half){
				// straight
				straightCount++;
			} else {
				// turn after straight
				if(firstFlag){
					machine.pushTarget(Position(0,CellWidth/2.f,0), new MotionLinear(new EasingTrap()), p_faststraight_start);
					if(straightCount>0){
						machine.pushTarget(pPivot, new MotionLinear(new EasingTrap()), p_faststraight);
					}
					firstFlag=false;
				} else if(straightCount>0){
					machine.pushTarget(pPivot, new MotionLinear(new EasingTrap()), p_faststraight);
				}
				machine.pushTarget(pTo, new MotionSmoothArc(new EasingLinear()), p_fastturn);
				straightCount=0;
			}

			// goal
			if(nextIndex==g_goal){
				if(straightCount>0){
					machine.pushTarget(pTo, new MotionLinear(new EasingTrap()), p_faststraight);
				}
				machine.pushTargetDiff(Position(-CellWidth/2.f*sin(pTo.angle),CellWidth/2.f*cos(pTo.angle),0.f), new MotionLinear(new EasingTrap()), p_faststraight_end);
			}
			// }}}
		}

		// prepare for the next step
		g_dir.half=agent.getDir().half;
		g_angle=normalized(pTo.angle,PI);
		g_index=nextIndex;
	}
	//machine.setWallAdjust();

	while(1){
		if(!machine.isActivated()){
			playErrorSound();
			//flushLog();
			return -1;
		}
		if(machine.isTargetSequenceEmpty()){
			//flushLog();
			break;
		}
		//flushLog();
		HAL_Delay(20);
	}
	playEndSound();
	//machine.resetAdjustment();
	HAL_Delay(2000);
	//machine.enableLog();
	//flushLog();
	//machine.setWallEdgeCorrection(false);

	agent.reroute();
	isEndOfSequence = true;
	if(procSearch(maze.getWidth()-1)<0) {
		maze.replace(backupMaze);
		return -1;
	}

	//machine.resetWallAdjust();
	machine.pushTargetDiff(Position(-CellWidth/2.f*sin(g_angle),CellWidth/2.f*cos(g_angle),0.f), new MotionLinear(new EasingTrap()), p_straight_end);
	HAL_Delay(3000);
	playConfirmSound();
	machine.deactivate();
	//flushLog();
	machine.block();
	return 0;
}

int8_t trajMode(){
	using namespace MyMath;
	using namespace MyMath::Machine;
	using namespace Trajectory;
	//machine.setWallCorrection(false);

	uint8_t param=modeSelect(7);
	if(param==0) return 0;

	/*
	p_faststraight_start = pa_faststraight_start[param-1];
	p_faststraight = pa_faststraight[param-1];
	p_faststraight_end = pa_faststraight_end[param-1];
	p_fastturn = pa_fastturn[param-1];
	*/

	if(waitIR()<0) return 0;
	playStartSound();
	HAL_Delay(4000);

	initialiseRun();
	//machine.setWallCorrection(true);

	switch(param){
		case 1:
			machine.pushTargetDiff(Position(0,0,-PI), new MotionTurn(new EasingPoly5()), p_miniturn);
			break;
		case 2:
			machine.pushTarget(Position(0,CellWidth/2+PreTurnDistance,0), new MotionLinear(new EasingTrap()), p_straight_start);
			for (uint8_t i = 0; i < 3; ++i) {
				machine.pushTarget(Position(CellWidth/2-PreTurnDistance,CellWidth,-MyMath::PI/2), new MotionSmoothArc(new EasingLinear()), p_turn);
				machine.pushTarget(Position(CellWidth/2+PreTurnDistance,CellWidth,-MyMath::PI/2), new MotionLinear(new EasingTrap()), p_straight);
				machine.pushTarget(Position(CellWidth,CellWidth/2+PreTurnDistance,MyMath::PI), new MotionSmoothArc(new EasingLinear()), p_turn);
				machine.pushTarget(Position(CellWidth,CellWidth/2-PreTurnDistance,MyMath::PI), new MotionLinear(new EasingTrap()), p_straight);
				machine.pushTarget(Position(CellWidth/2+PreTurnDistance,0,MyMath::PI/2), new MotionSmoothArc(new EasingLinear()), p_turn);
				machine.pushTarget(Position(CellWidth/2-PreTurnDistance,0,MyMath::PI/2), new MotionLinear(new EasingTrap()), p_straight);
				machine.pushTarget(Position(0,CellWidth/2-PreTurnDistance,0), new MotionSmoothArc(new EasingLinear()), p_turn);
				machine.pushTarget(Position(0,CellWidth/2+PreTurnDistance,0), new MotionLinear(new EasingTrap()), p_straight);
			}
			machine.pushTarget(Position(0,CellWidth,0), new MotionLinear(new EasingTrap()), p_straight_end);
			break;
		case 3:
			machine.pushTarget(Position(0,CellWidth/2,0), new MotionLinear(new EasingTrap()), p_faststraight_start);
			machine.pushTarget(Position(0,CellWidth*3,0), new MotionLinear(new EasingTrap()), p_faststraight);
			machine.pushTarget(Position(0,CellWidth*3+CellWidth/2,0), new MotionLinear(new EasingTrap()), p_faststraight_end);
			break;
		case 4:
			machine.pushTarget(Position(0,CellWidth/2,0), new MotionLinear(new EasingTrap()), p_ministraight);
			break;
		case 5:
			for (uint8_t i = 0; i < 20; ++i) {
				machine.pushTargetDiff(Position(0.f,0.f,PI), new MotionTurn(new EasingPoly5()), p_miniturn);
			}
			break;
		case 6:
			for (uint8_t i = 0; i < 20; ++i) {
				machine.pushTargetDiff(Position(0.f,0.f,-PI), new MotionTurn(new EasingPoly5()), p_miniturn);
			}
			break;
		default:
			break;
	}

	while(1){
		if(!machine.isActivated()){
			playErrorSound();
			//flushLog();
			return -1;
		}
		if(machine.isTargetSequenceEmpty()){
			playEndSound();
			//playErrorSound();
			//flushLog();
			break;
		}
		//flushLog();
		HAL_Delay(20);
	}
	HAL_Delay(3000);
	playConfirmSound();
	machine.deactivate();
	//flushLog();
	return 0;
}

void runMode(){
	uint8_t mode=modeSelect(6); int8_t result=0;
	switch(mode){
		case 0:  return;
		case 1:  result=searchRunMode(); break;
		case 2:  result=fastRunMode(true, true); break;
		case 3:  result=fastRunMode(false, true); break;
		case 4:  result=searchRunMode(true); break;
		case 5:  return;//result=fastRunMode(true, false); break;
		case 6:  result=trajMode(); break;
		default: break;
	}
	/*
	switch(result){
		case -1: buzzer; break;
		default: break;
	}
	*/
	return;
}

void sensorMode(){
	machine.unblock();
	char cbuf[100];
	while(1){
		uint8_t ccnt = sprintf(cbuf, "%4d, %4d, %4d\n", machine.getADCValue(IRSensor::LF), machine.getADCValue(IRSensor::FR), machine.getADCValue(IRSensor::RF));
		HAL_UART_Transmit(&huart1, cbuf, ccnt, 100);
		button.updateState();
		if(button.getState()==Button::RisingEdge){
			machine.block();
			playConfirmSound();
			return;
		}
		HAL_Delay(10);
	}
}

void goalSetMode(){
	uint8_t gx = modeSelect(32);
	playErrorSound();
	uint8_t gy = modeSelect(32);
	playErrorSound();
	maze.setGoal(gx, gy);
	backupMaze.setGoal(gx, gy);
	return;
}

void selectHalfMode(){
	using namespace MyMath::Machine;
	PillarWidth = 6.f;
	CellWidth = 90.f;
	RearLength = 23.f+PillarWidth/2.f;
	InitialY = RearLength-CellWidth/2.f;
	PreTurnDistance = 10.f;
	PreSensDistance = 10.f;
	CellRatio = 0.5f;
	playErrorSound();
}

void selectClassicMode(){
	using namespace MyMath::Machine;
	PillarWidth = 12.f;
	CellWidth = 180.f;
	RearLength = 23.f+PillarWidth/2.f;
	InitialY = RearLength-CellWidth/2.f;
	PreTurnDistance = 45.f;
	CellRatio = 1.f;
	playMario();
}

void selectQuarterMode(){
	using namespace MyMath::Machine;
	PillarWidth = 3.f;
	CellWidth = 45.f;
	RearLength = 23.f+PillarWidth/2.f;
	InitialY = RearLength-CellWidth/2.f;
	PreTurnDistance = 10.f;
	CellRatio = 0.25f;
	playMario();
}

void sizeModeMenu(){
	uint8_t mode=modeSelect(3);
	switch(mode){
		case 0:  return;
		case 1:  selectQuarterMode(); return;
		case 2:  selectHalfMode(); return;
		case 3:  selectClassicMode(); return;
		default: break;
	}
	return;
}

void dumpMaze(MazeSolver::Maze& m){
	for(int8_t i=maze.getHeight()-1;i>=0;i--){
		for(uint8_t j=0;j<maze.getWidth();j++){
			char buf[6];
			uint8_t d = maze.getCellData(j,i).half;
			uint8_t cn = sprintf(buf, "%x ", d&0xf);
			HAL_UART_Transmit(&huart1, buf, cn, 100);
		}
		HAL_UART_Transmit(&huart1, "\n", 1, 100);
	}
}

void mazeDumpMenu(){
	uint8_t mode=modeSelect(2);
	switch(mode){
		case 0:  return;
		case 1:  dumpMaze(maze); return;
		case 2:  dumpMaze(backupMaze); return;
		default: break;
	}
	return;
}

void menu(){
	uint8_t mode=modeSelect(5);
	switch(mode){
		case 0:  return;
		case 1:  return runMode();
		case 2:  return sensorMode();
		case 3:  return mazeDumpMenu();
		case 4:  sizeModeMenu(); return;//circuitMode(); return;
		case 5:  goalSetMode(); return;
		default: break;
	}
	return;
}

void wait(){
	while(1){
		machine.block();
		button.updateState();
		if(button.getState()==Button::RisingEdge){
			// TODO: implement menu
			playConfirmSound(); menu();
		}
		HAL_Delay(20);
	}
}

int main(void)
{
	HAL_Init();

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

	MX_GPIO_Init();

	// NVIC (at least) should be configured after HAL_Init
	irsensor.configIRQ();
	machine.configIRQ();
	irled1.configIRQ();
	irled2.configIRQ();

	MX_USART1_UART_Init();

	__HAL_DBGMCU_FREEZE_TIM1();
	__HAL_DBGMCU_FREEZE_TIM2();

	encoderL.reset(); encoderR.reset();

	leds.set(0);
	leds.set(1);
	leds.set(2);
	leds.set(3);
	leds.set(4);
	leds.set(5);

	HAL_Delay(500);

	if(!imu1.test()){ leds.reset(3); leds.reset(4); leds.reset(5); }
	if(!imu2.test()){ leds.reset(3); leds.reset(4); leds.reset(5); }

	if(!imu1.init()){ leds.reset(3); leds.reset(4); leds.reset(5); }
	if(!imu2.init()){ leds.reset(3); leds.reset(4); leds.reset(5); }

	uint16_t buf[6];
	uint32_t batt_sum = 0;
	for(uint8_t i=0; i<200; i++){
		irsensor.startConv();
		HAL_Delay(1);
		while(!irsensor.isCompleted()){}
		irsensor.stopConv();
		irsensor.resetCompleted();
		irsensor.readValues(buf);
		batt_sum += buf[4];
	}
	if(batt_sum/200 <= 2250){
		playErrorSound();
		while(1);
	}

	MazeLoader::loadEmpty(32, 32, 1, 0, maze);

	backupMaze.replace(maze);

	graph.loadEmpty(32,32);
	graph.loadMaze(&maze);
	graph.reset();

	playBootSound();

	machine.block();

	wait();
}

void test_IMUs(){
	// Testing IMUs
	volatile int16_t x = imu1.readInt16(59);
	volatile int16_t x2 = imu2.readInt16(59);
	if(x<0){ leds.reset(0); leds.set(1); }
	else { leds.reset(1); leds.set(0); }
	if(x2<0){ leds.reset(2); leds.set(3); }
	else { leds.reset(3); leds.set(2); }
}

void test_encoders(){
	// Testing encoders
	encoderL.captureSpeed();
	if(encoderL.speed()<0){ leds.reset(0); leds.set(1); }
	else if(encoderL.speed()==0) {  }
	else { leds.reset(1); leds.set(0); }
	encoderR.captureSpeed();
	if(encoderR.speed()<0){ leds.reset(2); leds.set(3); }
	else if(encoderR.speed()==0) {  }
	else { leds.reset(3); leds.set(2); }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
		|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }

	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}
/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 921600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) { _Error_Handler(__FILE__, __LINE__); }
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) { while(1){} }

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line){}

#endif

__weak HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /*Configure the SysTick to have interrupt in 0.5ms time basis*/
  HAL_SYSTICK_Config(SystemCoreClock/2000);

  /*Configure the SysTick IRQ priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority ,0);

   /* Return function status */
  return HAL_OK;
}


#ifdef __cplusplus
}
#endif
