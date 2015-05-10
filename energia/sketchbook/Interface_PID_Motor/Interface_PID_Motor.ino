/*
 *	Luis Alfredo Servín Garduño
 *	National and Autonomous University of Mexico
 *	Project: @Work
 * 	Speed motor control
 *	
 *	Motor characteristics:
 *	This program is use to work with the next model of motor
 *	75:1 Metal Gearmotor 25Dx54L mm HP with 48 CPR Encoder
 *	the encoder provides 3592 counts per revolution. Operation
 *	range is 3- 9 V intended to for use at 6 V 
 *
 *	Key aspects at 6v: 130 RPM and 450 mA free-run, 130 oz-in
 *	(9.4 kg-cm) and 6 A stall
 *
 *	The motor/encoder color-code is:
 *	Red 	  -	Motor power
 *	Black	  -	Motor power
 *	Green 	-	Encoder GND
 *	Blue 	  - Encoder Vcc (3.5 - 20 V)
 *	Yellow	- Encoder A output
 *	White 	-	Encoder B output
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

// pins for motor1
byte pwm1Pin = 25;
byte en1Pin = 26;
byte en2Pin = 27;

byte encm1pin1 = 7;
byte encm1pin2 = 8;


// pins for motor2
byte pwm2Pin = 36;
byte en3Pin = 34;
byte en4Pin = 35;

byte encm2pin1 = 13;
byte encm2pin2 = 14;

// encoder caracteristics 
//int to float
const float countsPerRevolution = 3592.0;

// variable to store the value from the encoder
volatile long encPos1 = 0;
volatile long encPos2 = 0;

float motorAng1=0.0;
float motorAng2=0.0;
long time_last_1=0;
long time_last_2=0;
float motorVel1=0.0;
float motorVel2=0.0;
int umbral_vel=20.0; //modify this value 
float erorvel=0.0;
float dt_1=0;
float dt_2=0;
float errorVel_Last_1=0;
float errorVel_Last_2=0;
const int motor_range_vel=3.0;

//Constants for control
float kisum_vel=0.0;
const int  kp_vel=2.0;
const int ki_vel=5.0;
const int kd_vel=5.0;
//const for windup
const int windup_sup=5;
const int windup_inf=1;
const int kimax_vel=2.0;


// Kinematic model characteristics
float lRobot = 2.0;

int pwmM1;
int pwmM2;

int pwm;

//#define PI=3.141592;

void messageCb(const geometry_msgs::Twist& pwm_vel)
{
  // calculate speed according to the kinematic model
  pwmM1 = pwm_vel.linear.x * 255 + (lRobot / 2) * pwm_vel.angular.z * 255;
  pwmM2 = pwm_vel.linear.x * 255 - (lRobot / 2) * pwm_vel.angular.z * 255;

  // set the direction of rotation
  setRotationDirection(pwmM1,pwmM2);
  //Controlling velocity of motors
  //motor 1
  pwmM1=Control_Motor('1',encPos1,motorAng1,dt_1, time_last_1, pwmM1,errorVel_Last_1);
  //motor 2
  pwmM2=Control_Motor('2',encPos2,motorAng2,dt_2, time_last_2, pwmM2,errorVel_Last_2);

  analogWrite(pwm1Pin, pwmM1);
  analogWrite(pwm2Pin, pwmM2);
}

//ros variables
ros::NodeHandle nh;
// create a instance for the message type Vector 3
geometry_msgs::Vector3 encWheelTicks;
std_msgs::Int32 intvel_msg;
// Create a publisher with the following characteristics:
// Object name: chatter
// Topic:       motVel
// Type:        Int16 (%intvel_msg reference to the message instance to be published)
ros::Publisher chatter("encTicks",&encWheelTicks);
// Create a subscriber with the following characteristics
// Object name: sub
// Topic:       cmd_vel
// Function:    messageCb
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

// void messageCb(const std_msgs::& pwm_vel){


void setup() {
  // Declare pins for motor control
  pinMode(en1Pin, OUTPUT);
  pinMode(en2Pin, OUTPUT);
  pinMode(en3Pin, OUTPUT);
  pinMode(en4Pin, OUTPUT);

  // Declare pins for encoder control
  pinMode(encm1pin1, INPUT);
  pinMode(encm1pin2, INPUT);
  pinMode(encm2pin1, INPUT);
  pinMode(encm2pin2, INPUT);

  // Set interrupts for every encoder's pin
  attachInterrupt(encm1pin1, enc1m1change, CHANGE);
  attachInterrupt(encm1pin2, enc2m1change, CHANGE);
  attachInterrupt(encm2pin1, enc1m2change, CHANGE);
  attachInterrupt(encm2pin2, enc2m2change, CHANGE);

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop() {
  if(millis() % 50 == 0) {
    encWheelTicks.y = encPos1;
    encWheelTicks.x = encPos2;
    chatter.publish(&encWheelTicks);
    encPos1 = 0;
    encPos2 = 0;
    delay(1);
  }
  nh.spinOnce();
}

/*
 * Functinos to read encoders are based on:
 * http://playground.arduino.cc/Main/RotaryEncoders
 */

void enc1m1change()
{
  if(digitalRead(encm1pin1) == HIGH) {
    if(digitalRead(encm1pin2) == LOW) {
      encPos1 += 1;
    } else {
      encPos1 -= 1;
    }
  } else {
    if(digitalRead(encm1pin2) == HIGH) {
      encPos1 += 1;
    } else {
      encPos1 -= 1;
    }
  }
}

void enc2m1change()
{
  if(digitalRead(encm1pin2) == HIGH) {
    if(digitalRead(encm1pin1) == HIGH) {
      encPos1 += 1;
    } else {
      encPos1 -= 1;
    }
  } else {
    if(digitalRead(encm1pin1) == LOW) {
      encPos1 += 1;
    } else {
      encPos1 -= 1;
    }
  }
}
void enc1m2change()
{
  if(digitalRead(encm2pin1) == HIGH) {
    if(digitalRead(encm2pin2) == LOW) {
      encPos2 += 1;
    } else {
      encPos2 -= 1;
    }
  } else {
    if(digitalRead(encm2pin2) == HIGH) {
      encPos2 += 1;
    } else {
      encPos2 -= 1;
    }
  }
}
void enc2m2change()
{
  if(digitalRead(encm2pin2) == HIGH) {
    if(digitalRead(encm2pin1) == HIGH) {
      encPos2 += 1;
    } else {
      encPos2 -= 1;
    }
  } else {
    if(digitalRead(encm2pin1) == LOW) {
      encPos2 += 1;
    } else {
      encPos2 -= 1;
    }
  }
}
void setRotationDirection(float pwmM1, float pwmM2)
{  if(pwmM1 > 0) {
    digitalWrite(en1Pin, HIGH);
    digitalWrite(en2Pin, LOW);
  } else {
    digitalWrite(en1Pin, LOW);
    digitalWrite(en2Pin, HIGH);
  }

  if(pwmM2 > 0) {
    digitalWrite(en3Pin, HIGH);
    digitalWrite(en4Pin, LOW);
  } else {
    digitalWrite(en3Pin, LOW);
    digitalWrite(en4Pin, HIGH);
  }
}

//This is the rutine for control one motor at time
int Control_Motor(char mot,long int encPos, float motorAng, float dt, long int time_last, float motorVelDes,float errorVel_Last)
{
  //calculate the velocities and angles
    float motorVel=angCalc(mot,encPos,motorAng,time_last); 
   //WE HAVE TO TRANSLATE DE VELOCITIES
    float pwmM=PID_vel(motorVel,motorVelDes, dt, errorVel_Last);

  // get the absolute value
    pwmM = int(abs(pwmM));
    pwmM=constrain(pwmM2,0,255);


 return pwmM; 
}
float angCalc(char mot, long int encPos, float motorAng, long int time_last)
{
   float motorAng_Last=motorAng;
   motorAng=float(encPos)*2.0*PI/countsPerRevolution;
   float dt=(time_last-millis())/1000;    
   float motorVel=(motorAng-motorAng_Last)/dt;//calculete the velocity

    if(mot=='1')//set the value for motor 1
      {motorAng1=motorAng;
        time_last_1=millis();
        dt_1=dt;}
   else//set the value for motor 2
      {motorAng2=motorAng; 
       time_last_2=millis();
        dt_2=dt;
      }
return motorVel;
}
float PID_vel(float motorVel,float motorVelDes, float dt, float errorVel_Last)
{
  float errorVel;
  //error-tolerant
  if (abs(motorVelDes-motorVel)<umbral_vel)
      errorVel=0;
  else
      errorVel=motorVelDes-motorVel+0.0;
  
  if(abs(errorVel)<windup_sup)
        {
          if(abs(errorVel)<windup_inf) //betewen umbral and windup
            kisum_vel=0;//The integral part of PID has no value in this part
          else //The integral part has effect in the control the windup effect
            { 
            kisum_vel+=ki_vel*errorVel;
            kisum_vel=constrain(kisum_vel, -kimax_vel, kimax_vel);//We limit the value of kisum
            }
            
        }
  else
      kisum_vel=0; 

  float motorOut=constrain(kp_vel*errorVel+kisum_vel+kd_vel*((errorVel-errorVel_Last)/(dt)), -motor_range_vel, motor_range_vel);
  return motorOut; 
}
        


