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
// here the pinmode in made inside instance creation
motor Mot1(M1_DIR, M1_PWM, M1_ENC1, M1_ENC2);

// local variables
int angPosition = 0;

void setup()
{
  // initialize serial communication for debugging
  Serial.begin(BAUD_RATE);

  // attach neccesary interruptions for reading motor encoders
  attachInterrupt(M1_ENC1, iM1E1, CHANGE);
  attachInterrupt(M1_ENC2, iM1E2, CHANGE);

  // set pwm and direction for motor
  Mot1.setPwm(50);
  Mot1.setDirection(LOW);
}

void loop()
{
  // write electronic values
  Mot1.runMotor();
  delay(500);

  angPosition = Mot1.stepsIncrement();
  Serial.println(angPosition);
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