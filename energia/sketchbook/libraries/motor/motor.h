/**
 * motor.h - Library for representing a motor object
 * Created by Luis A. Servin
 * Released into public domain
 */

#ifndef motor_h
#define motor_h

#include "Arduino.h"

class motor
{
public:

	/**
	 * Signal for controlling the motor
	 */
	int pwm;
	bool direction;

	/** 
	 * Physical properties
	 */
	int countsPerRevolution;

	/**
	 * Constructor. As arguments it only need physical pins.
	 */
	motor(int pinDirection, int pinPWM, int pinEncoderA, int pinEncoderB);

	/** 
	 * Set pwm value for motor.
	 */
	void setPwm(int Pwm);

	/**
	 * Get pwm description
	 * @return [int] pwm actual value
	 */
	int getPwm();

	/**
	 * Set roation direction
	 * @param Direction 1 for forward
	 */
	void setDirection(bool Direction);

	/** 
	 * Changes actual electronic values
	 */
	void runMotor();

	/**
	 * Get differential encoder steps since last reset. 
	 * @return [long] encoder steps since last call.
	 */
	long getEncoderSteps();

	/**
	 * Function for calculating quadatrure increments for encoder A.
	 */
	void doEncoderA();

	/**
	 * Function for calculating quadatrure increments for encoder B.
	 */
	void doEncoderB();

	/**
	 * Function for calculating differential encoder.
	 * @return [int] Increment in encoder since last call
	 */
	int stepsIncrement();

	float calculateAngularPosition();
	float calculateAngularSpeed();
	float calculateAbsoluteAngularPosition();
	
private:  
	/**
	 * Variables for saving encoder position
	 */
	volatile long _encoderSteps;
	long _lastEncoderPosition;
	long _actualEncoderPosition;
	int _lastSteps;

	/**
	 * Pins for physical connection
	 */
	int _pinDirection;
	int _pinPWM;
	int _pinEncoderA;
	int _pinEncoderB;

	float _angularSpeed;
	float _currentAngularPosition;
	int _currentTime;
	int _lastTime;
};

#endif // motor_h
