/*
Copyright (c) 2016 TU Delft All Rights Reserved.
*/

/**********/
/* Header */
/**********/
// Libraries
#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_MAX31865.h>

//TODO: in target
#define MAX31865_FAULT_HIGHTHRESH   0x80
#define MAX31865_FAULT_LOWTHRESH    0x40
#define MAX31865_FAULT_REFINLOW     0x20
#define MAX31865_FAULT_REFINHIGH    0x10
#define MAX31865_FAULT_RTDINLOW     0x08
#define MAX31865_FAULT_OVUV   			0x04

// Pin numbers
#define MAX_BOTTOM		23
#define MAX_SIDE1			25
#define MAX_SIDE2			27
#define MAX_SIDE3			29
#define MAX_SIDE4			31
#define MAX_CENTER1		22
#define MAX_CENTER2		24
#define MAX_CENTER3		26
#define MAX_CENTER4		28
#define MAX_CENTER5		30

// Setup of MAX31865 PT100 amplifiers using hardware SPI
Adafruit_MAX31865 maxBottom = Adafruit_MAX31865(MAX_BOTTOM);
Adafruit_MAX31865 maxSide1 = Adafruit_MAX31865(MAX_SIDE1);
Adafruit_MAX31865 maxSide2 = Adafruit_MAX31865(MAX_SIDE2);
Adafruit_MAX31865 maxSide3 = Adafruit_MAX31865(MAX_SIDE3);
Adafruit_MAX31865 maxSide4 = Adafruit_MAX31865(MAX_SIDE4);
Adafruit_MAX31865 maxCenter1 = Adafruit_MAX31865(MAX_CENTER1);
Adafruit_MAX31865 maxCenter2 = Adafruit_MAX31865(MAX_CENTER2);
Adafruit_MAX31865 maxCenter3 = Adafruit_MAX31865(MAX_CENTER3);
Adafruit_MAX31865 maxCenter4 = Adafruit_MAX31865(MAX_CENTER4);
Adafruit_MAX31865 maxCenter5 = Adafruit_MAX31865(MAX_CENTER5);

// RTD variables
float temp1; float temp2; float temp3; float temp4; float temp5; float temp6; float temp7; float temp8; float temp9; float temp10;
uint16_t fault1; uint16_t fault2; uint16_t fault3; uint16_t fault4; uint16_t fault5; uint16_t fault6; uint16_t fault7; uint16_t fault8; uint16_t fault9; uint16_t fault10;

// Create array of 10 MAX31865 objects
const int maxN =  10;
Adafruit_MAX31865 maxNames [maxN] = {maxBottom, maxSide1, maxSide2, maxSide3, maxSide4, maxCenter1, maxCenter2, maxCenter3, maxCenter4, maxCenter5};
float tempNames [maxN] = {temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10};
uint16_t faultNames [maxN] = {fault1, fault2, fault3, fault4, fault5, fault6, fault7, fault8, fault9, fault10};

// The value of the Rref resistor
#define RREF 430.0
// #define CONVERSION 32768

int incomingByte;
bool displayFault = false;


/*********/
/* Setup */
/*********/
void setup() {
	// TODO: Serial.begin was 19200! change?
	Serial.begin(19200);

	// Setup all MAX31865 objects
	for (int i = 0; i < maxN; i++ ) {
		maxNames[i].begin(MAX31865_4WIRE);
	}
}


/********/
/* Loop */
/********/
void loop() {
	//Read signal from computer
	if (Serial.available() > 0) {
		// Read the incoming byte
		incomingByte = Serial.read();
		// Echo incoming Byte
		Serial.write(incomingByte);

		// Check wether to print Full error or not
		if(incomingByte == 'A') {
			displayFault = true;
		}
		else if(incomingByte == 'B') {
			displayFault = false;
		}
		if(incomingByte == 'S') {
			// Start
		}

		//Reset incomingbyte value
		incomingByte=' ';
	}

	// For every PT100 read the value and check errors
	//TODO: ++i of i++
	for (int i = 0; i < maxN; i++ ) {
		// Read Temperature value and Print
		tempNames[i] = maxNames[i].temperature(100, RREF);

		// Send Data to target
		sendData();

		// Check for errors
		checkFault(i);

		// Wait 10ms
		delay(10);
	}
	Serial.println();
	delay(1000);
}


/*************/
/* Functions */
/*************/
void sendData(){
	// Send the values of the center RTDs to target
	if(i >= 5){
		// Print Device to target
		Serial.print('D');
		Serial.print('\n');
		Serial.print((MAX_BOTTOM - 11) + (i * 2));
		Serial.print('\n');

		// Print Values to target
		Serial.print('V');
		Serial.print('\n');
		Serial.print(tempNames[i]);
		Serial.print('\n');
	}
	// else{
	// 	Serial.print('D');
	// 	Serial.print('\n');
	// 	Serial.print(MAX_BOTTOM + (i * 2));
	// 	Serial.print('\n');
	//
	// 	// Print Values to target
	// 	Serial.print('V');
	// 	Serial.print('\n');
	// 	Serial.print(tempNames[i]);
	// 	Serial.print('\n');
	// }
	// // Test print
	// Serial.print("Temperature "); Serial.print(22+i); Serial.print(": "); Serial.print(tempNames[i]);
}

void checkFault(int i){
	faultNames[i] = maxNames[i].readFault();
	if (faultNames[i]) {

		// // Test print
		// Serial.print("("); Serial.print("Fault 0x"); Serial.print(faultNames[i], HEX); Serial.print(") ");

		// Print Error to target
		Serial.print('E');
		Serial.print('\n');
		Serial.print(faultNames[i], HEX);
		Serial.print('\n');

		// TODO: receive error at target and check error value at target, thus eliminating this code
		if(displayFault){
			if (faultNames[i] & MAX31865_FAULT_HIGHTHRESH) {
				Serial.print("RTD High Threshold");
			}
			if (faultNames[i] & MAX31865_FAULT_LOWTHRESH) {
				Serial.print("RTD Low Threshold");
			}
			if (faultNames[i] & MAX31865_FAULT_REFINLOW) {
				Serial.print("REFIN- > 0.85 x Bias");
			}
			if (faultNames[i] & MAX31865_FAULT_REFINHIGH) {
				Serial.print("REFIN- < 0.85 x Bias - FORCE- open");
			}
			if (faultNames[i] & MAX31865_FAULT_RTDINLOW) {
				Serial.print("RTDIN- < 0.85 x Bias - FORCE- open");
			}
			if (faultNames[i] & MAX31865_FAULT_OVUV) {
				Serial.print("Under/Over voltage");
			}
		}
		maxNames[i].clearFault();
	}
}
