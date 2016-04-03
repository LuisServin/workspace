/**
 * pid.cpp - library for representing a pid control
 * Created by Luis A. Servin
 * Released into public domain
 */

#include "Arduino.h"
#include "velocityPID.h"

velocityPID::velocityPID()
{
	_kFeedForward = 0.0f;
	_kProportional = 0.0f;
	_kDifferential = 0.0f;
	_kIntegral = 0.0f;

	_windUpSuperior = 0;
	_windUpInferior = 0;
	_maximumPWMOutput = 0;
}

velocityPID::setControlGains(float kFeedForward, float kProportional,
	float kDifferential, float kIntegral)
{
	_kFeedForward = kFeedForward;
	_kProportional = kProportional;
	_kDifferential = kDifferential;
	_kIntegral = kIntegral;
}

velocityPID::setWindUpLimits(int superiorLimit, int inferiorLimit)
{
	_windUpSuperior = superiorLimit;
	_windUpInferior = inferiorLimit;
}

velocityPID::setMaximumOutput(int maximumPWMValue)
{
	_maximumPWMOutput = maximumPWMValue;
}