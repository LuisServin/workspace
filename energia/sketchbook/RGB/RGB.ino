// Luis Servin
// Controlling a RGB led with a bluetooth command.

#include <SoftwareSerial.h>

#define R_LED 9
#define G_LED 10
#define B_LED 11

#define bluetoothTx 7
#define bluetoothRx 8

int pinOutRGB[3] = {R_LED, G_LED, B_LED};
int valuesRGB[3] = {0, 0, 0};

// create SoftwareSerial object
SoftwareSerial bT(bluetoothTx, bluetoothRx);

void setup() {
  // Start serial communication
  Serial.begin(9600);  

  // Start bluetooth communication
  bT.begin(9600);
}

void loop() {
  // set random values for RGB
  if(bT.available() == 3){
  	for(int i = 0; i<3; i++){
  		valuesRGB[i] = bT.read();
  		Serial.println(valuesRGB[i]);
  	}
  	setLedColor(pinOutRGB, valuesRGB);
  }
}

void setLedColor(int pinOut[3], int values[3]) {
	for(int i = 0; i<3; i++){
		analogWrite(pinOut[i], values[i]);
	}
}
