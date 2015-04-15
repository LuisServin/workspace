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


// Kinematic model characteristics
float lRobot = 2.0;

int pwmM1;
int pwmM2;

int pwm;


void messageCb(const geometry_msgs::Twist& pwm_vel)
{
  // calculate speed according to the kinematic model
  pwmM1 = pwm_vel.linear.x * 255 + (lRobot / 2) * pwm_vel.angular.z * 255;
  pwmM2 = pwm_vel.linear.x * 255 - (lRobot / 2) * pwm_vel.angular.z * 255;

  // set the direction of rotation
  if(pwmM1 > 0) {
    digitalWrite(en1Pin, HIGH);
    digitalWrite(en2Pin, LOW);
    digitalWrite(en5Pin,HIGH);
    digitalWrite(en6Pin,LOW);
  } else {
    digitalWrite(en1Pin, LOW);
    digitalWrite(en2Pin, HIGH);
    digitalWrite(en5Pin,LOW);
    digitalWrite(en6Pin,,HIGH);
  }

  if(pwmM2 > 0) {
    digitalWrite(en3Pin, HIGH);
    digitalWrite(en4Pin, LOW);
    digitalWrite(en7Pin,HIGH);
    digitalWrite(en8Pin,LOW);
  } else {
    digitalWrite(en3Pin, LOW);
    digitalWrite(en4Pin, HIGH);
    digitalWrite(en7Pin,LOW);
    digitalWrite(en8Pin,HIGH);
  }

  // get the absolute value
  pwmM1 = abs(pwmM1);
  pwmM2 = abs(pwmM2);

  // set a upper limit
  if(pwmM1 > 255)
    pwmM1 = 255;
  if(pwmM2 > 255)
    pwmM2 = 255;

  analogWrite(pwm1Pin, pwmM1);
  analogWrite(pwm2Pin, pwmM2);
  analogWrite(pwm3Pin,pwmM1);
  analogWrite(pwm4Pin,pwmM2);
}

//ros variables
ros::NodeHandle nh;
// create a instance for the message type Vector 3
geometry_msgs::Vector3 encVec;
std_msgs::Int32 intvel_msg;
// Create a publisher with the following characteristics:
// Object name: chatter
// Topic:       motVel
// Type:        Int16 (%intvel_msg reference to the message instance to be published)
ros::Publisher chatter("encTicks",&encVec);
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
  attachInterrupt(encm3pin1, enc1m3change, CHANGE);
  attachInterrupt(encm3pin2, enc2m3change, CHANGE);
  attachInterrupt(encm4pin1, enc1m4change, CHANGE);
  attachInterrupt(encm4pin2, enc2m4change, CHANGE);

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

}

void loop() {
  if(millis() % 200 == 0) {
    encVec.x = encPos1;
    encVec.y = encPos2;
    chatter.publish(&encVec);
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


//rutinas de encoders 3 y 4

void enc1m3change()
{
  if(digitalRead(encm3pin1) == HIGH) {
    if(digitalRead(encm3pin2) == LOW) {
      encPos3 += 1;
    } else {
      encPos3 -= 1;
    }
  } else {
    if(digitalRead(encm3pin2) == HIGH) {
      encPos3 += 1;
    } else {
      encPos3 -= 1;
    }
  }
}

void enc2m3change()
{
  if(digitalRead(encm3pin2) == HIGH) {
    if(digitalRead(encm3pin1) == HIGH) {
      encPos3 += 1;
    } else {
      encPos3 -= 1;
    }
  } else {
    if(digitalRead(encm3pin1) == LOW) {
      encPos3 += 1;
    } else {
      encPos3 -= 1;
    }
  }
}
  
void enc1m4change()
{
  if(digitalRead(encm4pin1) == HIGH) {
    if(digitalRead(encm4pin2) == LOW) {
      encPos4 += 1;
    } else {
      encPos4 -= 1;
    }
  } else {
    if(digitalRead(encm4pin2) == HIGH) {
      encPos4 += 1;
    } else {
      encPos4 -= 1;
    }
  }
}

void enc2m4change()
{
  if(digitalRead(encm4pin2) == HIGH) {
    if(digitalRead(encm4pin1) == HIGH) {
      encPos4 += 1;
    } else {
      encPos4 -= 1;
    }
  } else {
    if(digitalRead(encm4pin1) == LOW) {
      encPos4 += 1;
    } else {
      encPos4 -= 1;
    }
  }
}

