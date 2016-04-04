/**
 * Program for controlling a NeveRest motor using
 * classes
 */

#include "motor.h"
#include "velocityPID.h"

#define BAUD_RATE 9600

#define M1_PWM   23
#define M1_DIR   24
#define M1_ENC2  25
#define M1_ENC1  26

float TARGET_SPEED = 1.0;

volatile long pos1 = 0;

// Create the first motor object.
// here the pinmode in made inside instance creation
Motor         Mot1(M1_DIR, M1_PWM, M1_ENC1, M1_ENC2);

// Create an object for pid control
VelocityPID   Mot1_pid;

// local variables
long steps = 0;
float as;

/**************** SETUP *******************/

void setup()
{
  // initialize serial communication for debugging
  Serial.begin(BAUD_RATE);

  // attach neccesary interruptions for reading motor encoders
  attachInterrupt(M1_ENC1, iM1E1, CHANGE);
  attachInterrupt(M1_ENC2, iM1E2, CHANGE);

  // set pwm and direction for motor
  Mot1.setPwm(0);
  Mot1.setDirection(HIGH);
  // write electronic values
  Mot1.runMotor();

  // configutring pid control.
  Mot1_pid.setControlGains(0.0, 1.90, 0.1, 0.0);
}

/**************** LOOP *******************/

void loop()
{
  if(Serial.available()){
    TARGET_SPEED = Serial.read() - 48;
    //Serial.println(TARGET_SPEED);
    while(Serial.available()){
      Serial.read();
    }
  }

  delay(10);
  Mot1.calculateKinematicVariables();
  as = Mot1.getAngularSpeedRad();
  Serial.println(as);
  Mot1.setPwm(Mot1_pid.updatePID(Mot1.getPwm(), TARGET_SPEED, as));
  Mot1.runMotor();
}

// assign interruption function for every encoder respectively
// motor one
void iM1E1()
{
  Mot1.doEncoderA();
}

void iM1E2()
{
  Mot1.doEncoderB();
}