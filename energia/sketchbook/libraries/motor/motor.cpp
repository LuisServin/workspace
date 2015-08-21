/**
 * motor.cpp - Library for representing a motor object
 * Created by Luis A. Servin
 * Released into public domain
 */

#include "Arduino.h"
#include "motor.h"

/**
 * Constructor
 */

motor::motor(int pinDirection, int pinPWM, int pinEncoderA, int pinEncoderB)
{
	_pinDirection = pinDirection;
	_pinPWM = pinPWM;
	_pinEncoderA = pinEncoderA;
	_pinEncoderB = pinEncoderB;	

	pinMode(_pinDirection, OUTPUT);

	pinMode(_pinEncoderA, INPUT);
	pinMode(_pinEncoderB, INPUT);


	// Initialization of variables
	this->_angularSpeed = 0.0f;
	this->_currentAngularPosition = 0.0f;
	this->_currentTime = millis();
	this->_lastTime = 0;
	this->_encoderSteps = 0;
	this->countsPerRevolution = 3500;
}

void motor::setPwm(int Pwm)
{
	pwm = Pwm;
}

int motor::getPwm()
{
	return pwm;
}

void motor::setSpeed(int Pwm)
{
	pwm = Pwm;
}

int motor::getSpeed()
{
	return pwm;
}

void motor::setDirection(bool Direction)
{
	direction = Direction;
}

void motor::runMotor()
{
	digitalWrite(_pinDirection, direction);
	analogWrite(_pinPWM, pwm);
}

long motor::getEncoderSteps()
{
	return _encoderSteps;
}

void motor::doEncoderA()
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

void motor::doEncoderB()
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

float motor::calculateAngularSpeed()
{
	float angularSpeed, dt;

	float lastAngularPosition = this->_currentAngularPosition;
	this->_lastTime = this->_currentTime;

	this->_currentAngularPosition = calculateAngularPosition();
	this->_currentTime = millis();

	dt = float(this->_currentTime - this->_lastTime) / 1000;

	// Calculate angular speed
	if(this->direction && 
		(this->_currentAngularPosition < lastAngularPosition)) {
		angularSpeed = (((2 * PI) - lastAngularPosition) + 
			this->_currentAngularPosition) / dt;		
	} else {
		if(!this->direction && 
			(this->_currentAngularPosition > lastAngularPosition)){
			angularSpeed = (this->_currentAngularPosition - 
				(2 * PI + lastAngularPosition)) / dt;					
		} else {
			angularSpeed = (this->_currentAngularPosition - lastAngularPosition) / dt;
		}
	}

	return angularSpeed;
}

int motor::stepPosition()
{
	int steps = this->_encoderSteps;
	this->_encoderSteps = 0;

	int absoluteSteps = (steps + this->_lastSteps) 
		% this->countsPerRevolution;
	return absoluteSteps;
}

float motor::calculateAngularPosition()
{
	int absoluteSteps = stepPosition();
	float angularPosition = 
		float(absoluteSteps) * 2.0 * PI / this->countsPerRevolution;
	this->_lastSteps = absoluteSteps;
	return angularPosition;
}

float motor::calculateAbsoluteAngularPosition()
{
	float angularPosition;
	angularPosition = calculateAngularPosition();
	if(angularPosition < 0){
		angularPosition = 2 * PI + angularPosition;
		return angularPosition;
	}
	return angularPosition;
}