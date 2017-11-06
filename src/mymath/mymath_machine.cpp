#include "mymath_machine.h"

namespace MyMath { namespace Machine {
	/* Unit conversion functions */
	float convertPulseToWheelRadian(int32_t pulse){
		return (float)pulse*WheelRadianPerPulse;
	}
	float convertPulseToWheelDegree(int32_t pulse){
		return (float)pulse*WheelDegreePerPulse;
	}
	float convertPulseToWheelDistance(int32_t pulse){
		return (float)pulse*WheelDistancePerPulse;
	}
	float convertWheelDistanceToPulse(float distance){
		return distance*PulsePerWheelDistance;
	}
	float convertPulseDiffToMachineRadian(int32_t diff){
		return (float)diff*MachineRadianPerPulseDiff;
	}
	float convertMachineRadianToPulseDiff(float radian){
		return radian*PulseDiffPerMachineRadian;
	}
	float convertPulseDiffToMachineDegree(int32_t diff){
		return (float)diff*MachineDegreePerPulseDiff;
	}
	float convertMachineDegreeToPulseDiff(float degree){
		return degree*PulseDiffPerMachineDegree;
	}
	float convertGyroValueToMachineRadian(int32_t gyro){
		return (float)gyro*MachineRadianPerGyroValue;
	}
	float convertMachineRadianToGyroValue(float radian){
		return radian*GyroValuePerMachineRadian;
	}
	float convertGyroValueToMachineDegree(int32_t gyro){
		return (float)gyro*MachineDegreePerGyroValue;
	}
	float convertMachineDegreeToGyroValue(float degree){
		return degree*GyroValuePerMachineDegree;
	}
	float convertGyroValueToPulseDiff(int32_t gyro){
		return (float)gyro*PulseDiffPerGyroValue;
	}

	const Trajectory::Position convertRealPositionToMachinePosition(Trajectory::Position rp){
		Trajectory::Position mp;
		if(rp.angle < -PI) rp.angle += 2.f*PI;
		if(rp.angle > PI) rp.angle -= 2.f*PI;
		mp.setVars(
				convertWheelDistanceToPulse(rp.x),
				convertWheelDistanceToPulse(rp.y),
				convertMachineRadianToGyroValue(rp.angle)
				);
		return mp;
	}
} }
