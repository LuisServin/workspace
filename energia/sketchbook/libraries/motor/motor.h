/**
 * motor.h - Library for representing a motor object
 * Created by Luis A. Servin
 * Released into public domain
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "Arduino.h"

class Motor
{
public:
	/**
	 * Constructor. As arguments it only need physical pins.
	 */
	Motor(int pinDirection, int pinPWM, int pinEncoderA, int pinEncoderB);

	/** 
	 * Set pwm value for motor. Value will change until run motor
	 * function be called
	 */
	void setPwm(int pwm);

	/**
	 * Get pwm applied to moroe
	 * @return [int] actual pwm
	 */
	int getPwm();

	/**
	 * Set roation direction
	 * @param Direction 1 for forward
	 */
	void setDirection(bool direction);

	/** 
	 * Apply buffered values
	 */
	void runMotor();

	/**
	 * Get differential encoder steps since last reset, without reseting
	 * actual value 
	 * @return [long] encoder steps since last call.
	 */
	long getEncoderSteps();

	/**
	 * calculate actual kinematics like speed needed  for a control loop
	 * values are reset everytime this function is called
	 */
	void calculateKinematicVariables();

	/**
	 * Return speed in [steps/s]
	 * @return [int] angular speed
	 */
	int getAngularSpeedSteps();

	/**
	 * Return speed in [rad/s]
	 * @return [description]
	 */
	float getAngularSpeedRad();

	/**
	 * Function for calculating quadatrure increments for encoder A.
	 */
	void doEncoderA();

	/**
	 * Function for calculating quadatrure increments for encoder B.
	 */
	void doEncoderB();

	
private:	
	//Signal for controlling the motor
	int 	_pwm;
	bool 	_direction;

	//Physical properties
	int 	_countsPerRevolution;
	float 	_angularSpeed_steps;
	float 	_angularSpeed_rad;
	long 	_actualSteps;
	float 	_currentAngularPosition;
	unsigned long 	_currentTime;
	unsigned long 	_lastTime;

	//Variables for saving encoder position
	volatile long _encoderSteps;

	//Pins for physical connection
	int _pinDirection;
	int _pinPWM;
	int _pinEncoderA;
	int _pinEncoderB;

	/**
	 * Function for calculating differential encoder. this function reset
	 * actual values.
	 * @return [int] Increment in encoder since last call
	 */
	int stepsIncremented();

	float calculateAngularSpeedSteps(int steps);

	float calculateAngularSpeedRad(float steps_s);
};

#endif // MOTOR_H_
