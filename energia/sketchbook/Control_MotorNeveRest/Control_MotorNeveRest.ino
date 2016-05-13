/**
 * Created by Luis A. Servin
 * Released into public domain
 * Program for controlling a NeveRest 40 motor using
 * classes
 */

#include "motor.h"
#include "velocityPID.h"
#include <Thread.h>

// Baud rate for serial communication
#define BAUD_RATE 9600

// define for motors physical connections
#define M1_PWM   23
#define M1_DIR   24
#define M1_ENC1  26
#define M1_ENC2  25

#define M2_PWM   7
#define M2_DIR   8
#define M2_ENC1  6
#define M2_ENC2  5

// define loop intervals for different threads
#define CONTROL_LOOP_INTERVAL 10
#define SERIAL_LOOP_INTERVAL 1000

// defining control gains
#define MOTOR_FRONT_LEFT_KF 0.0
#define MOTOR_FRONT_LEFT_KP 1.9
#define MOTOR_FRONT_LEFT_KD 0.1
#define MOTOR_FRONT_LEFT_KI 0.0

#define MOTOR_FRONT_RIGHT_KF 0.0
#define MOTOR_FRONT_RIGHT_KP 1.9
#define MOTOR_FRONT_RIGHT_KD 0.1
#define MOTOR_FRONT_RIGHT_KI 0.0

float TARGET_SPEED = 1.0;

// first motor and pid control object.
Motor         MotorFrontLeft(M1_DIR, M1_PWM, M1_ENC1, M1_ENC2);
VelocityPID   MotorFrontLeft_pid;

// second motor and pid control object
Motor         MotorFrontRight(M2_DIR, M2_PWM, M2_ENC1, M2_ENC2);
VelocityPID   MotorFrontRight_pid;

// create a Thread for writting actual speed values
Thread serialThread = Thread();

// create a Thread for control loop
Thread controlThread = Thread();

// local variables
long steps = 0;
float as_1, as_2;

/*** CALLBACK FUNCTIONS ***/

// Serial Callback
void serialCb()
{
  // print actual values for motors speed in 
  Serial.print("MFR speed: ");
  Serial.print(MotorFrontLeft.getAngularSpeedSteps());
  Serial.print(" MFL speed: ");
  Serial.println(MotorFrontRight.getAngularSpeedSteps());
}

// Control Loop Callback
void controlCb()
{
  // control loop
  MotorFrontLeft.calculateKinematicVariables();
  MotorFrontRight.calculateKinematicVariables();

  as_1 = MotorFrontLeft.getAngularSpeedRad();
  as_2 = MotorFrontRight.getAngularSpeedRad();

  MotorFrontLeft.setPwm(
    MotorFrontLeft_pid.updatePID(MotorFrontLeft.getPwm(),
      TARGET_SPEED,
      as_1
    )
  );
  MotorFrontLeft.runMotor();

  MotorFrontRight.setPwm(
    MotorFrontRight_pid.updatePID(MotorFrontRight.getPwm(),
      TARGET_SPEED,
      as_2)
  );
  MotorFrontRight.runMotor();
}

/**************** SETUP *******************/

void setup()
{
  // initialize serial communication for debugging
  Serial.begin(BAUD_RATE);

  // attach neccesary interruptions for reading motors encoders
  attachInterrupt(M1_ENC1, iMotorFrontLeftE1, CHANGE);
  attachInterrupt(M1_ENC2, iMotorFrontLeftE2, CHANGE);

  attachInterrupt(M2_ENC1, iMotorFrontRightE1, CHANGE);
  attachInterrupt(M2_ENC2, iMotorFrontRightE2, CHANGE);

  // set initials pwm and direction for motors
  MotorFrontLeft.setPwm(0);
  MotorFrontLeft.setDirection(LOW);

  MotorFrontRight.setPwm(0);
  MotorFrontRight.setDirection(HIGH);

  // write electronic values
  MotorFrontLeft.runMotor();
  MotorFrontRight.runMotor();

  // configutring pid control.
  MotorFrontLeft_pid.setControlGains(MOTOR_FRONT_LEFT_KP, 
    MOTOR_FRONT_LEFT_KI, MOTOR_FRONT_LEFT_KD
  );
  MotorFrontRight_pid.setControlGains(MOTOR_FRONT_RIGHT_KP, 
    MOTOR_FRONT_RIGHT_KI, MOTOR_FRONT_RIGHT_KD
  );

  // configuring serialcb thread
  serialThread.onRun(serialCb);
  serialThread.setInterval(SERIAL_LOOP_INTERVAL);

  //configuring control thread
  controlThread.onRun(controlCb);
  controlThread.setInterval(CONTROL_LOOP_INTERVAL);
}

/**************** LOOP *******************/

void loop()
{
  // check if threads should run
  if(serialThread.shouldRun())
    serialThread.run();
  if(controlThread.shouldRun())
    controlThread.run();

  // check for a new target speed
  if(Serial.available()){
    TARGET_SPEED = Serial.parseFloat();
    Serial.println(TARGET_SPEED);
    while(Serial.available()){
      Serial.read();
    }
  }

  // delay for control
  // delay(10);
}

// assign interruption function for every encoder respectively
// motor one
void iMotorFrontLeftE1()
{
  MotorFrontLeft.doEncoderA();
}

void iMotorFrontLeftE2()
{
  MotorFrontLeft.doEncoderB();
}

// interrupt functions motor front right
void iMotorFrontRightE1()
{
  MotorFrontRight.doEncoderA();
}

void iMotorFrontRightE2()
{
  MotorFrontRight.doEncoderB();
}