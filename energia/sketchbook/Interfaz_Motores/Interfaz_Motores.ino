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

// pins for motor1
byte pwm1Pin = 36;
byte en1Pin = 37;
byte en2Pin = 38;

byte encm1pin1 = 39;
byte encm1pin2 = 40;


// pins for motor2
byte pwm2Pin = 35;
byte en3Pin = 33;
byte en4Pin = 34;

byte encm2pin1 = 11;
byte encm2pin2 = 12;

// encoder caracteristics
int countsPerRevolution = 3592;

// variable to store the value from the encoder
volatile long encPos1 = 0;
volatile long encPos2 = 0;

int pwm;

void messageCb(const geometry_msgs::Twist& pwm_vel)
{
  if(pwm_vel.linear.x > 0) {
    digitalWrite(en3Pin, HIGH);
    digitalWrite(en4Pin, LOW);
  } else {
    digitalWrite(en3Pin, LOW);
    digitalWrite(en4Pin, HIGH);
  }
  pwm = abs(pwm_vel.linear.x * 255);
  if(pwm > 255)
    pwm = 255;
  analogWrite(pwm2Pin, pwm);
  //analogWrite(pwm2Pin, pwm);
}

//ros variables
ros::NodeHandle nh;
// create a instance for the message type String
std_msgs::Int32 intvel_msg;
// Create a publisher with the following characteristics:
// Object name: chatter
// Topic:       motVel
// Type:        Int16 (%intvel_msg reference to the message instance to be published)
ros::Publisher chatter("motVel",&intvel_msg);
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
  if(millis() % 200 == 0) {
    intvel_msg.data = encPos2;
    chatter.publish(&intvel_msg);
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