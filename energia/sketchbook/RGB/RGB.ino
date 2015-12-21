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
#define BLUETOOTH_TX 7
#define BLUETOOTH_RX 8

// varibale for controlling delay time inside fade function
#define DELAY_TIME_FADE 2
#define DELAY_TIME_RANDOM 500

//Serial speed
#define SERIAL_BAUD 9600
#define BLUETOOTH_BAUD 9600

int pinOutRGB[3] = {R_LED, G_LED, B_LED};
int valuesRGB[3] = {0, 0, 0};

// create SoftwareSerial object
SoftwareSerial bT(BLUETOOTH_TX, BLUETOOTH_RX);

void setup() {
  // Start serial communication
  Serial.begin(SERIAL_BAUD);  

  // Start bluetooth communication
  bT.begin(BLUETOOTH_BAUD);
  
  pinMode(STATUS_LED, OUTPUT);
}

void loop() {
  // set random values for RGB by 
  // a bluetooth command
  if(bT.available() == 3){
    for(int i = 0; i<3; i++){
      valuesRGB[i] = bT.read();
      Serial.println(valuesRGB[i]);
    }
    setLedColor(pinOutRGB, valuesRGB);

    digitalWrite(STATUS_LED, HIGH);
    delay(8000);
    digitalWrite(STATUS_LED, LOW);
  } else {
    // if there is not a bluetooth command
    // make another effect
    fadeLedRGB(pinOutRGB);
    randomLedRGB(pinOutRGB, 5);
  }
}

/**
 * set a specific color to a RGB led based on a 
 * user specification
 */
void setLedColor(int pinOut[3], int values[3]) {
	for(int i = 0; i<2; i++){
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
          values[i] = random(0, 255);
      }
      for(int j=0; j<3; j++){
        analogWrite(pinOut[i], values[i]);
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
