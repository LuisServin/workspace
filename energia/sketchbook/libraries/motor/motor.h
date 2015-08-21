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
	/*Signal for controlling the motor*/
	int pwm;
	bool direction;
	/*Physical properties*/
	int countsPerRevolution;

	motor(int pinDirection, int pinPWM, int pinEncoderA, int pinEncoderB);
	void setPwm(int Pwm);
	int getPwm();
	void setSpeed(int Speed);	// [m/s]
	int getSpeed();
	void setDirection(bool Direction);
	void runMotor();
	long getEncoderSteps();
	void doEncoderA();
	void doEncoderB();

	int stepPosition();
	float calculateAngularPosition();
	float calculateAngularSpeed();
	float calculateAbsoluteAngularPosition();

private:
	volatile long _encoderSteps;
	int _lastSteps;
	/*Pins for physical connection*/
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
