/**
 * pid.cpp - library for representing a pid control
 * Created by Luis A. Servin
 * Released into public domain
 */

#include "Arduino.h"
#include "velocityPID.h"
VelocityPID::VelocityPID()
{
	_kFeedForward = 0.0f;
	_kProportional = 0.0f;
	_kDifferential = 0.0f;
	_kIntegral = 0.0f;

	_windUpSuperior = 0;
	_windUpInferior = 0;
	_maximumPWMOutput = 0;
}

void VelocityPID::setControlGains(float kFeedForward, float kProportional,
	float kDifferential, float kIntegral)
{
	_kFeedForward = kFeedForward;
	_kProportional = kProportional;
	_kDifferential = kDifferential;
	_kIntegral = kIntegral;
}

void VelocityPID::setWindUpLimits(int superiorLimit, int inferiorLimit)
{
	_windUpSuperior = superiorLimit;
	_windUpInferior = inferiorLimit;
}

void VelocityPID::setMaximumOutput(int maximumPWMValue)
{
	_maximumPWMOutput = maximumPWMValue;
}

int VelocityPID::updatePID(float actOutput, float targetValue, 
	float actualValue)
{
	float pid = 0;
	float error = 0;
	static float last_error = 0;

	error = abs(targetValue) - abs(actualValue);
	pid = (_kProportional * error) + 
		(_kDifferential * (error - last_error));
	//store last error
	last_error = error;

	//return actOutput + pid;
	return constrain(actOutput + int(pid), 0, 255);
}