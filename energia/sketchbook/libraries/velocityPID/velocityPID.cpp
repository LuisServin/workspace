/**
 * pid.cpp - library for representing a pid control
 * Created by Luis A. Servin
 * Released into public domain
 */

#include "Arduino.h"
#include "velocityPID.h"

velocityPID::velocityPID()
{
	this->_kFeedForward = 0.0f;
	this->_kProportional = 0.0f;
	this->_kDifferential = 0.0f;
	this->_kIntegral = 0.0f;

	this->_windUpSuperior = 0;
	this->_windUpInferior = 0;
	this->_maximumPWMOutput = 0;
}

velocityPID::setControlGains(float kFeedForward, float kProportional,
	float kDifferential, float kIntegral)
{
	this->_kFeedForward = kFeedForward;
	this->_kProportional = kProportional;
	this->_kDifferential = kDifferential;
	this->_kIntegral = kIntegral;
}

velocityPID::setWindUpLimits(int superiorLimit, int inferiorLimit)
{
	this->_windUpSuperior = superiorLimit;
	this->_windUpInferior = inferiorLimit;
}

velocityPID::setMaximumOutput(int maximumPWMValue)
{
	this->_maximumPWMOutput = maximumPWMValue;
}