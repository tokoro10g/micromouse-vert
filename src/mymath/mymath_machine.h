#pragma once

#include "mymath.h"
#include "../libtrajectory/target.h"

namespace MyMath { namespace Machine {
	/* Constants */
	static const float GearRatio = 40.f/9.f;
	static const uint16_t EncoderPulse = 2000;
	static const float WheelPulse = (float)EncoderPulse;

	static const float WheelDiameter = 13.3f;
	static const float WheelPerimeter = WheelDiameter*PI;

	static const float WheelRadianPerPulse = PI/(WheelPulse/2.f);
	static const float PulsePerWheelRadian = 1.f/WheelRadianPerPulse;

	static const float WheelDegreePerPulse = 180.f/(WheelPulse/2.f);
	static const float PulsePerWheelDegree = 1.f/WheelDegreePerPulse;

	static const float WheelDistancePerPulse = WheelPerimeter/WheelPulse;
	static const float PulsePerWheelDistance = 1.f/WheelDistancePerPulse;

	static const float MachineWidth = 24.5f;
	static const float WheelWidth = 3.f;
	static const float TreadWidth = MachineWidth-WheelWidth;

	static const float PillarWidth = 6.f;
	static const float CellWidth = 90.f;

	static const float RearLength = 20.f+PillarWidth/2.f;
	static const float InitialY = RearLength-CellWidth/2.f;

	static const float PulseDiffPerMachineRadian = TreadWidth*PulsePerWheelDistance;
	static const float MachineRadianPerPulseDiff = 1.f/PulseDiffPerMachineRadian;

	static const float MachineDegreePerPulseDiff = 180.f/(PI*TreadWidth*PulsePerWheelDistance);
	static const float PulseDiffPerMachineDegree = 1.f/MachineDegreePerPulseDiff;

	static const float GyroValuePerMachineRotation = 16.4f*360.f*2000.f; // calculated value

	static const float GyroValuePerMachineRadian = GyroValuePerMachineRotation/2.f/PI;
	static const float MachineRadianPerGyroValue = 1.f/GyroValuePerMachineRadian;

	static const float GyroValuePerMachineDegree = GyroValuePerMachineRotation/360.f;
	static const float MachineDegreePerGyroValue = 1.f/GyroValuePerMachineDegree;

	static const float GyroValuePerPulseDiff = GyroValuePerMachineRadian*MachineRadianPerPulseDiff;
	static const float PulseDiffPerGyroValue = 1.f/GyroValuePerPulseDiff;

	static const float PreTurnDistance = 4.f;

	/* Unit conversion functions */
	float convertPulseToWheelRadian(int32_t pulse);
	float convertPulseToWheelDegree(int32_t pulse);
	float convertPulseToWheelDistance(int32_t pulse);
	float convertWheelDistanceToPulse(float distance);
	float convertPulseDiffToMachineRadian(int32_t diff);
	float convertMachineRadianToPulseDiff(float radian);
	float convertPulseDiffToMachineDegree(int32_t diff);
	float convertMachineDegreeToPulseDiff(float degree);
	float convertGyroValueToMachineRadian(int32_t gyro);
	float convertMachineRadianToGyroValue(float radian);
	float convertGyroValueToMachineDegree(int32_t gyro);
	float convertMachineDegreeToGyroValue(float degree);
	float convertGyroValueToPulseDiff(int32_t gyro);

	static const float PIInPulseDiff = PI*PulseDiffPerMachineRadian;
	static const float PIInGyroValue = PI*GyroValuePerMachineRadian;

	const Trajectory::Position convertRealPositionToMachinePosition(Trajectory::Position rp);
}}
