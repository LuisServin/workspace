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

//#define USE_USBCON

#include <ros.h>
#include <atwork_msgs/FourWheels.h>

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


// pins for motor3
byte pwm3Pin = 30;
byte en5Pin = 29;
byte en6Pin = 28;

byte encm3pin1 = 9;
byte encm3pin2 = 10;

// pins for motor4
byte pwm4Pin = 31;
byte en7Pin = 33;
byte en8Pin = 32;

byte encm4pin1 = 11;
byte encm4pin2 = 12;

// encoder caracteristics
int countsPerRevolution = 3592;

// variable to store the value from the encoder
volatile long encPos1 = 0;
volatile long encPos2 = 0;
volatile long encPos3 = 0;
volatile long encPos4 = 0;

int pwmM1 = 0;
int pwmM2 = 0;
int pwmM3 = 0;
int pwmM4 = 0;

void wheelSpeedCb(const atwork_msgs::FourWheels& data)
{
	pwmM1 = data.wheel1 * 25;
	pwmM2 = data.wheel2 * 25;
	pwmM3 = data.wheel3 * 25;
	pwmM4 = data.wheel4 * 25;

	if(pwmM1 > 0) {
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

	if(pwmM3 > 0) {
		digitalWrite(en5Pin, HIGH);
		digitalWrite(en6Pin, LOW);
	} else {
		digitalWrite(en5Pin, LOW);
		digitalWrite(en6Pin, HIGH);
	}

	if(pwmM4 > 0) {
		digitalWrite(en7Pin, HIGH);
		digitalWrite(en8Pin, LOW);
	} else {
		digitalWrite(en7Pin, LOW);
		digitalWrite(en8Pin, HIGH);
	}

	pwmM1 = abs(pwmM1);
	pwmM2 = abs(pwmM2);
	pwmM3 = abs(pwmM3);
	pwmM4 = abs(pwmM4);

	if(pwmM1 > 255)
		pwmM1 = 255;
	if(pwmM2 > 255)
		pwmM2 = 255;
	if(pwmM3 > 255)
		pwmM3 = 255;
	if(pwmM4 > 255)
		pwmM4 = 255;

	analogWrite(pwm1Pin, pwmM1);
	analogWrite(pwm2Pin, pwmM2);
	analogWrite(pwm3Pin, pwmM3);
	analogWrite(pwm4Pin, pwmM4);
}

//ros variables
ros::NodeHandle nh;

// Create a subscriber with the following characteristics
// Object name: wheelSpeedSub
// Topic:       wheelSpeeds
// Function:    messageCb
ros::Subscriber<atwork_msgs::FourWheels> wheelSpeedSub("wheelSpeeds", &wheelSpeedCb);

void setup()
{
  // Declare pins for motor control
  pinMode(en1Pin, OUTPUT);
  pinMode(en2Pin, OUTPUT);
  pinMode(en3Pin, OUTPUT);
  pinMode(en4Pin, OUTPUT);
  pinMode(en5Pin, OUTPUT);
  pinMode(en6Pin, OUTPUT);
  pinMode(en7Pin, OUTPUT);
  pinMode(en8Pin, OUTPUT);

  // Declare pins for encoder control
  pinMode(encm1pin1, INPUT);
  pinMode(encm1pin2, INPUT);
  pinMode(encm2pin1, INPUT);
  pinMode(encm2pin2, INPUT);

  nh.initNode();
  //nh.advertise(chatter);
  nh.subscribe(wheelSpeedSub);
}

void loop()
{
  nh.spinOnce(); 
}
