/**
 * pid.cpp - library for representing a pid control
 * Created by Luis A. Servin
 * Released into public domain
 */

#include "Arduino.h"
#include "velocityPID.h"
VelocityPID::VelocityPID()
{
	// default values 
	_kFeedForward = 0.0f;
	_kProportional = 1.0f;
	_kIntegral = 0.0f;
	_kDifferential = 0.0f;

	_windUpLimit = 50;
	_maximumPWMOutput = 255;
}

void VelocityPID::setControlGains(float kProportional,
	float kDifferential, float kIntegral)
{
	_kProportional = kProportional;
	_kDifferential = kDifferential;
	_kIntegral = kIntegral;
}

void VelocityPID::setWindUpLimits(int Limit)
{
	_windUpLimit = Limit;
}

void VelocityPID::setMaximumOutput(int maximumPWMValue)
{
	_maximumPWMOutput = maximumPWMValue;
}

int VelocityPID::updatePID(float actualOutput, float targetValue, 
	float actualValue)
{
	float PID = 0;
	float error = 0;

	// declaring terms for pid control
	static float P = 0;
	static float I = 0; 
	static float D = 0;
	static float last_error = 0;
	static float Integral = 0;

	// calculating error 
	error = abs(targetValue) - abs(actualValue);

	// evaluating integrla term and avoiding wind up
	if (abs(Integral) < _windUpLimit){
		Integral += error;
	} else  {
		Integral = 0;
	}
		

	P = error * _kProportional;
	I = Integral * _kIntegral;
	D = (last_error - error) * _kDifferential;
 
	PID = P + I + D;

	//store last error
	last_error = error;

	// return pid value and constrain it to 0 - 255
	return constrain(actualOutput + int(PID), 0, _maximumPWMOutput);
}