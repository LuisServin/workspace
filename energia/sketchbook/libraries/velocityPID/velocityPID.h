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
	/**
	 * Constructor: Create a velocity PID object
	 */
	VelocityPID();

	/**
	 * Set control gains for PID control
	 * @param kProportional Proportional gain
	 * @param kIntegral     Integral gain
	 * @param kDifferential Differential gain
	 */
	void setControlGains(float kProportional, float kIntegral,
		float kDifferential);

	/**
	 * Set windup limits for integral term. Based on this
	 * implementation only a windup is need
	 * @param Limit windup limit, absolute value will be used
	 *              for both limits
	 */
	void setWindUpLimits(int Limit);

	/** Establish limit for PWM value output or 
	 * 	control output, by default in almost any uC is 255
	 */
	void setMaximumOutput(int maximumPWMValue);

	/**
	 * Calculate pid value
	 * @param  actualOutput Actual pwm value. needed for velocity pid
	 * @param  targetValue  Target variable value
	 * @param  actualValue  Actual variable value
	 * @return              New pwm value
	 */
	int updatePID(float actualOutput, float targetValue, float actualValue);

private:
	/**
	 * pid gains
	 */
	float _kFeedForward; // Gain for feedforward the variable
	float _kProportional;
	float _kDifferential;
	float _kIntegral;
	
	/**
	 * limits for calculating pid and output values 
	 */
	int _windUpLimit;	//This is compared to the error
	int _maximumPWMOutput;
};

#endif // VELOCITY_PID_H_