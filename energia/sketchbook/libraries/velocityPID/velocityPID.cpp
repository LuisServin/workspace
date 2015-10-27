/**
 * pid.cpp - library for representing a pid control
 * Created by Luis A. Servin
 * Released into public domain
 */

#include "Arduino.h"
#include "velocityPID.h"

velocityPID::velocityPID()
{
	this->kFeedForward = 0.0f;
	this->kProportional = 0.0f;
	this->kDifferential = 0.0f;
	this->kIntegral = 0.0f;

	this->windUpSuperior = 0;
	this->windUpInferior = 0;
	this->maximumPWMOutput = 0;
}

velocityPID::setControlGains(float kFeedForward, float kProportional,
	float kDifferential, float kIntegral)
{
	this->kFeedForward = kFeedForward;
	this->kProportional = kProportional;
	this->kDifferential = kDifferential;
	this->kIntegral = kIntegral;
}

velocityPID::setWindUpLimits(int superiorLimit, int inferiorLimit)
{
	this->windUpSuperior = superiorLimit;
	this->windUpInferior = inferiorLimit;
}

velocityPID::setMaximumOutput(int maximumPWMValue)
{
	this->maximumPWMOutput = maximumPWMValue;
}