/**
 * pid.h - library for representing a pid control
 * Created by Luis A. Servin
 * Released into public domain
 * based on: http://forum.arduino.cc/index.php?topic=8652.0
 */

#ifndef VELOCITY_PID_H_
#define VELOCITY_PID_H_

#include "Arduino.h"

class VelocityPID
{
public:
	VelocityPID();
	void setControlGains(float kFeedForward, float kProportional,
		float kIntegral, float kDifferential);
	void setWindUpLimits(int superiorLimit, int inferiorLimit);
	void setMaximumOutput(int maximumPWMValue);
	int updatePID(float actOutput, float targetValue, float actualValue);

private:
	float _kFeedForward; // Gain for feedforward the variable
	float _kProportional;
	float _kDifferential;
	float _kIntegral;
	
	int _windUpSuperior;	//This is compared to the error
	int _windUpInferior;
	int _maximumPWMOutput;
};

#endif // VELOCITY_PID_H_