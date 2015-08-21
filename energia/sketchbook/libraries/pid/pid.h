/**
 * pid.h - library for representing a pid control
 * Created by Luis A. Servin
 * Released into public domain
 */

#ifndef pid_h
#define pid_h

#include "Arduino.h"

class pid
{
public:
	pid();
	~pid();
private:
	float _kisum;
	int _kp;
	int _kd;
	int _ki;
	int _km;

	int _windup_sup;
	int _windup_inf;
	int _kimax_vel;
};

#endif // pid_h
