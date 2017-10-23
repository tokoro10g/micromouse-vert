#pragma once

#include "mymath.h"

namespace MyMath { namespace Machine {
	/* Constants */
	static const float GearRatio = 53.f/13.f;
	static const uint16_t EncoderPulse = 2048;
	static const float WheelPulse = GearRatio*(float)EncoderPulse;

	static const float WheelDiameter = 24.33f;
	static const float WheelPerimeter = WheelDiameter*PI;

	static const float WheelRadianPerPulse = PI/(WheelPulse/2.f);
	static const float PulsePerWheelRadian = 1.f/WheelRadianPerPulse;

	static const float WheelDegreePerPulse = 180.f/(WheelPulse/2.f);
	static const float PulsePerWheelDegree = 1.f/WheelDegreePerPulse;

	static const float WheelDistancePerPulse = WheelPerimeter/WheelPulse;
	static const float PulsePerWheelDistance = 1.f/WheelDistancePerPulse;

	static const float MachineWidth = 73.3f;
	static const float WheelWidth = 7.5f;
	static const float TreadWidth = 74.3082f;
	//static const float TreadWidth = 68.6818f;
	//static const float TreadWidth = MachineWidth-WheelWidth;
	static const float RearLength = 32.f+6.f;
	static const float InitialY = RearLength-90.f;

	static const float PulseDiffPerMachineRadian = TreadWidth*PulsePerWheelDistance;
	static const float MachineRadianPerPulseDiff = 1.f/PulseDiffPerMachineRadian;

	static const float MachineDegreePerPulseDiff = 180.f/(PI*TreadWidth*PulsePerWheelDistance);
	static const float PulseDiffPerMachineDegree = 1.f/MachineDegreePerPulseDiff;

	//static const float GyroValuePerMachineRotation = 36935.0;
	//static const float GyroValuePerMachineRotation = 37165.0;
	//static const float GyroValuePerMachineRotation = 37195.0;
	//static const float GyroValuePerMachineRotation = 37165.0;
	//static const float GyroValuePerMachineRotation = 73800.f;
	//static const float GyroValuePerMachineRotation = 73850.f;
	static const float GyroValuePerMachineRotation = 73728.f; // calculated value
	//static const float GyroValuePerMachineRotation = 74630.f;

	static const float GyroValuePerMachineRadian = GyroValuePerMachineRotation/PI;
	static const float MachineRadianPerGyroValue = 1.f/GyroValuePerMachineRadian;

	static const float GyroValuePerMachineDegree = GyroValuePerMachineRotation/180.f;
	static const float MachineDegreePerGyroValue = 1.f/GyroValuePerMachineDegree;

	static const float GyroValuePerPulseDiff = GyroValuePerMachineRadian*MachineRadianPerPulseDiff;
	static const float PulseDiffPerGyroValue = 1.f/GyroValuePerPulseDiff;

	static const float PreTurnDistance = 7.f;
	//static const float PreTurnDistance = 15.f;

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

	/*
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
	*/

	float PIInPulseDiff = PI*PulseDiffPerMachineRadian;
	float PIInGyroValue = PI*GyroValuePerMachineRadian;

}}
