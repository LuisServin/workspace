// Luis Servin
// Controlling a RGB led with a bluetooth command.
#include <SoftwareSerial.h>

// pinout used for RGB led
#define R_LED 9
#define G_LED 10
#define B_LED 11

#define STATUS_LED 13 

// using SoftwareSerial library to create a virtual
// serial port
#define BLUETOOTH_TX 14
#define BLUETOOTH_RX 8

// varibale for controlling delay time inside fade function
#define DELAY_TIME_FADE 2
#define DELAY_TIME_RANDOM 1000

// Serial speed
#define SERIAL_BAUD 9600
#define BLUETOOTH_BAUD 9600

// variables for controlling RGB led
int pinOutRGB[3] = {R_LED, G_LED, B_LED};
int valuesRGB[3] = {0, 0, 0};
bool userMode = false;
char userSelection = ' ';

// create SoftwareSerial object
// SoftwareSerial bT(BLUETOOTH_TX, BLUETOOTH_RX);

void setup() {
  // Start serial communication
  Serial.begin(SERIAL_BAUD);  

  // Start bluetooth communication
  //Serial1.begin(BLUETOOTH_BAUD);
  
  pinMode(STATUS_LED, OUTPUT);
}

void loop() {
  if(Serial.available()) {
    userSelection = Serial.read();
    //Serial.println(userSelection);
    if(userSelection == 's') {
      digitalWrite(13, HIGH);
      userMode = true;
      
      while(Serial.available() < 3) {
        // wait for three values
      }
      for(int i = 0; i<3; i++){
        valuesRGB[i] = Serial.read();
        //Serial.println(valuesRGB[i]);
      }
      cleanBTSerialPort();
      setLedColor(pinOutRGB, valuesRGB);

      digitalWrite(13, LOW);
    } else if(userSelection == 'r' || userSelection == 'f') {
      analogWrite(pinOutRGB[2], 255);
      userMode = false;
    }
  } else if(!userMode) {
    // if there is not a bluetooth command
    // make another effect
    for(int i=0; i<3; i++){
      analogWrite(pinOutRGB[i], 0);
    }
    if(userSelection == 'r') {
      randomLedRGB(pinOutRGB, 1);
    } else if(userSelection == 'f') {
      fadeLedRGB(pinOutRGB); 
    }
  }
}

/**
 * clean bluetooth buffer for remaining values
 */
void cleanBTSerialPort() {
  while(Serial1.available()){
      Serial1.read();
  }
}

/**
 * set a specific color to a RGB led based on a 
 * user specification
 */
void setLedColor(int pinOut[3], int values[3]) {
  for(int i = 0; i<3; i++){
    analogWrite(pinOut[i], values[i]);
  }
}

/**
 * Create a random values a specifig number of times
 * in a RGB led. Actual color is set only in an specific
 * time interval.
 */
void randomLedRGB(int pinOut[3], int numberTimes) {
  int values[3];

  // set colors
  for(int i=0; i<numberTimes; i++){
      for(int j=0; j<3; j++){
          values[j] = random(0, 255);
      }
      for(int j=0; j<3; j++){
        analogWrite(pinOut[j], values[j]);
      }
      delay(DELAY_TIME_RANDOM);
  }

  // clean RGB led
  for(int i=0; i<3; i++){
      analogWrite(pinOut[i], 0);
  }
}

/** 
 * function for make a rgb led fade incrementally
 * and decrementally in an interval set interval of time
 */ 
void fadeLedRGB(int pinOut[3]) {
  for(int i=0; i<3; i++){
      for(int j=0; j<255; j++){
        analogWrite(pinOut[i], j);
        delay(DELAY_TIME_FADE);
      }
      for(int j=255; j>-1; j--){
        analogWrite(pinOut[i], j);
        delay(DELAY_TIME_FADE);   
      }
  }
}
