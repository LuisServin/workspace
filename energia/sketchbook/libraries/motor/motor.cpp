/**
 * motor.cpp - Library for representing a motor object
 * Created by Luis A. Servin
 * Released into public domain
 */

#include "Arduino.h"
#include "motor.h"

#define PI 	3.14159

/**
 * Constructor
 */
Motor::Motor(int pinDirection, int pinPWM, int pinEncoderA, int pinEncoderB)
{
	// pins for physical connection
	_pinDirection = pinDirection;
	_pinPWM = pinPWM;
	_pinEncoderA = pinEncoderA;
	_pinEncoderB = pinEncoderB;	

	// initialize pins in uC
	pinMode(_pinDirection, OUTPUT);
	pinMode(_pinEncoderA, INPUT);
	pinMode(_pinEncoderB, INPUT);


	// Initialization of variables
	_countsPerRevolution = 3500;
	_encoderSteps = 0;
	
	_angularSpeed_steps = 0.0f;
	_angularSpeed_rad = 0.0f;
	_currentTime = 0;
	_lastTime = 0;
}

void Motor::setPwm(int pwm)
{
	_pwm = pwm;
}

void Motor::setDirection(bool direction)
{
	_direction = direction;
}

void Motor::runMotor()
{
	digitalWrite(_pinDirection, _direction);
	analogWrite(_pinPWM, _pwm);
}

void Motor::doEncoderA()
{
	if(digitalRead(_pinEncoderA) == HIGH) {
		if(digitalRead(_pinEncoderB) == HIGH) {
			_encoderSteps += 1;
		} else {
			_encoderSteps -= 1;
		} 
	} else {
		if(digitalRead(_pinEncoderB) == LOW) {
			_encoderSteps += 1;
		} else {
			_encoderSteps -= 1;
		}
	}
}

void Motor::doEncoderB()
{
	if(digitalRead(_pinEncoderB) == HIGH) {
		if(digitalRead(_pinEncoderA) == LOW) {
			_encoderSteps += 1;
		} else {
			_encoderSteps -= 1;
		}
	} else {
		if(digitalRead(_pinEncoderA) == HIGH) {
			_encoderSteps += 1;
		} else {
			_encoderSteps -= 1;
		}
	}
}

long Motor::getEncoderSteps()
{
	return _encoderSteps;
}

int Motor::stepsIncremented()
{
	// save increment and restore variable to avoid overloading
	int steps = abs(_encoderSteps);
	_encoderSteps = 0;
	return steps;
}

void Motor::calculateKinematicVariables()
{
	_actualSteps = stepsIncremented();
	_angularSpeed_steps = calculateAngularSpeedSteps(_actualSteps);
	_angularSpeed_rad = calculateAngularSpeedRad(_angularSpeed_steps);
}

int Motor::getAngularSpeedSteps()
{
	return _angularSpeed_steps;
}

float Motor::getAngularSpeedRad()
{
	return _angularSpeed_rad;
}

float Motor::calculateAngularSpeedSteps(int steps)
{
	float angularSpeed, dt;

	_lastTime = _currentTime;
	_currentTime = millis();
	dt = _currentTime - _lastTime; // [ms]
	if (dt != 0) {
		angularSpeed = steps / dt; // [steps / ms]
		// convert actual speed to [steps / s]
		angularSpeed *= 1000;
	} else {
		angularSpeed = -1;
	}

	return angularSpeed;
}

float Motor::calculateAngularSpeedRad(float steps_s)
{
	float angularSpeed_rad;
	angularSpeed_rad = steps_s * 2 * PI / _countsPerRevolution;

	return angularSpeed_rad;
}