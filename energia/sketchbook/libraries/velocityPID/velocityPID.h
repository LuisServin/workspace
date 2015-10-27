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
	float kFeedForward; // Gain for feedforward the variable
	float kProportional;
	float kDifferential;
	float kIntegral;

	int windUpSuperior;	//This is compared to the error
	int windUpInferior;
	int maximumPWMOutput;

	velocityPID();
	void setControlGains(float kFeedForward, float kProportional, float kDifferential, float kIntegral);
	void setWindUpLimits(int superiorLimit, int inferiorLimit);
	void setMaximumOutput(int maximumPWMValue);
	~pid();
private:
	
};

#endif // pid_h
