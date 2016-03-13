/**
 * pid.h - library for representing a pid control
 * Created by Luis A. Servin
 * Released into public domain
 */

#ifndef velocityPID_h
#define velocityPID_h

#include "Arduino.h"

class velocityPID
{
public:
	velocityPID();
	void setControlGains(float kFeedForward, float kProportional, float kDifferential, float kIntegral);
	void setWindUpLimits(int superiorLimit, int inferiorLimit);
	void setMaximumOutput(int maximumPWMValue);
	~pid();

private:
	float _kFeedForward; // Gain for feedforward the variable
	float _kProportional;
	float _kDifferential;
	float _kIntegral;
	
	int _windUpSuperior;	//This is compared to the error
	int _windUpInferior;
	int _maximumPWMOutput;
};

#endif // pid_h
