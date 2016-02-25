/**
 * Program for controlling a NeveRest motor using
 * classes
 */

#include <motor.h>

#define BAUD_RATE 9600

#define M1_PWM   23
#define M1_DIR   24
#define M1_ENC2  25
#define M1_ENC1  26

volatile long pos1 = 0;

// Create the first motor object.
motor Mot1(M1_DIR, M1_PWM, M1_ENC1, M1_ENC2);

int angSpeed = 0;

void setup()
{
  Serial.begin(BAUD_RATE);

  attachInterrupt(M1_ENC1, iM1E1, CHANGE);
  attachInterrupt(M1_ENC2, iM1E2, CHANGE);
  Mot1.setPwm(50);
  Mot1.setDirection(LOW);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Mot1.runMotor();
  delay(500);
  angSpeed = Mot1.stepIncrement();
  Serial.println(angSpeed);
}

void iM1E1()
{
  Mot1.doEncoderA();
}

void iM1E2()
{
  Mot1.doEncoderB();
}