/*
Copyright (c) 2016 TU Delft All Rights Reserved.
Rens Doornbusch
*/

/**********/
/* Header */
/**********/
/* Libraries */
#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_MAX31865.h>
#include <PID_v1.h>

/* Constants */
// Pin numbers
#define PIN_DC_SENSOR		A0

#define PIN_PWM_BOTTOM		2
#define PIN_PWM_SIDE1		3
#define PIN_PWM_SIDE2		4
#define PIN_PWM_SIDE3		5
#define PIN_PWM_SIDE4		6
#define PIN_PWM_CENTER		7

#define PIN_MAX_BOTTOM		23
#define PIN_MAX_SIDE1		25
#define PIN_MAX_SIDE2		27
#define PIN_MAX_SIDE3		29
#define PIN_MAX_SIDE4		31
#define PIN_MAX_CENTER1		22
#define PIN_MAX_CENTER2		24
#define PIN_MAX_CENTER3		26
#define PIN_MAX_CENTER4		28
#define PIN_MAX_CENTER5		30

// Resistance of MAX31865 Reference resitor and RTD at 0 degrees
const float resitanceReference 	= 430.0;
const int resitanceRTD 				= 100;

// Constants for voltmeter
const float voltageReference 		= 2.50;
const float voltageCompensation 	= 0.155;
const float resistorBig 			= 19820.0;
const float resistorSmall 			= 938.0;
const float analogResolution 		= 4095.0;

// number of PWM and MAX31865 devices
const int pwmN 						= 6;
const int maxN 						= 10;

/* Variables */
// TODO: Decrease globals

int incomingByte;

// voltmeter
float analogRaw = 0.0;
float voltageIn = 0.0;
float voltageSourceRaw = 0.0;
float voltageSourceCompensated = 0.0;

// PID variables
double tempTarget = 34.00;
float aggKp=2000, aggKi=0, aggKd=0;
float consKp=1500, consKi=0.01, consKd=0;

// PWM variables
double pwmOutput1; double pwmOutput2; double pwmOutput3; double pwmOutput4; double pwmOutput5; double pwmOutput6;
double pidInput1; double pidInput2; double pidInput3; double pidInput4; double pidInput5; double pidInput6;

// MAX31865 variables
float temp1; float temp2; float temp3; float temp4; float temp5; float temp6; float temp7; float temp8; float temp9; float temp10;
uint16_t fault1; uint16_t fault2; uint16_t fault3; uint16_t fault4; uint16_t fault5; uint16_t fault6; uint16_t fault7; uint16_t fault8; uint16_t fault9; uint16_t fault10;

/* Objects */
// Setup of MAX31865 PT100, RTD, amplifiers using hardware SPI
Adafruit_MAX31865 maxBottom 	= Adafruit_MAX31865(PIN_MAX_BOTTOM);
Adafruit_MAX31865 maxSide1 	= Adafruit_MAX31865(PIN_MAX_SIDE1);
Adafruit_MAX31865 maxSide2 	= Adafruit_MAX31865(PIN_MAX_SIDE2);
Adafruit_MAX31865 maxSide3 	= Adafruit_MAX31865(PIN_MAX_SIDE3);
Adafruit_MAX31865 maxSide4 	= Adafruit_MAX31865(PIN_MAX_SIDE4);
Adafruit_MAX31865 maxCenter1 	= Adafruit_MAX31865(PIN_MAX_CENTER1);
Adafruit_MAX31865 maxCenter2 	= Adafruit_MAX31865(PIN_MAX_CENTER2);
Adafruit_MAX31865 maxCenter3 	= Adafruit_MAX31865(PIN_MAX_CENTER3);
Adafruit_MAX31865 maxCenter4 	= Adafruit_MAX31865(PIN_MAX_CENTER4);
Adafruit_MAX31865 maxCenter5 	= Adafruit_MAX31865(PIN_MAX_CENTER5);

// Setup of PID controllers
PID myPID1(&pidInput1, &pwmOutput1, &tempTarget, aggKp, aggKi, aggKd, DIRECT);
PID myPID2(&pidInput2, &pwmOutput2, &tempTarget, aggKp, aggKi, aggKd, DIRECT);
PID myPID3(&pidInput3, &pwmOutput3, &tempTarget, aggKp, aggKi, aggKd, DIRECT);
PID myPID4(&pidInput4, &pwmOutput4, &tempTarget, aggKp, aggKi, aggKd, DIRECT);
PID myPID5(&pidInput5, &pwmOutput5, &tempTarget, aggKp, aggKi, aggKd, DIRECT);
PID myPID6(&pidInput6, &pwmOutput6, &tempTarget, aggKp, aggKi, aggKd, DIRECT);

// MAX31865  arrays
Adafruit_MAX31865 maxArray [maxN] = {maxBottom, maxSide1, maxSide2, maxSide3, maxSide4, maxCenter1, maxCenter2, maxCenter3, maxCenter4, maxCenter5};
float tempArray [maxN] = {temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10};
uint16_t faultArray [maxN] = {fault1, fault2, fault3, fault4, fault5, fault6, fault7, fault8, fault9, fault10};

// PWM arrays
PID pidArray [pwmN] = {myPID1, myPID2, myPID3, myPID4, myPID5, myPID6};
int pwmPinArray [pwmN] = {PIN_PWM_BOTTOM, PIN_PWM_SIDE1, PIN_PWM_SIDE2, PIN_PWM_SIDE3, PIN_PWM_SIDE4, PIN_PWM_CENTER};
double pidInputArray [pwmN] = {pidInput1, pidInput2, pidInput3, pidInput4, pidInput5, pidInput6};
double pwmOutputArray [pwmN] = {pwmOutput1, pwmOutput2, pwmOutput3, pwmOutput4, pwmOutput5, pwmOutput6};


/*********/
/* Setup */
/*********/
void setup() {
	// TODO: Serial.begin was 19200! change to faster?
	Serial.begin(19200);

	// Set resolution to 12bit (0-4095)
	analogWriteResolution(12);
	analogReadResolution(12);

	// Setup MAX31865 objects
	for (int i = 0; i < maxN; i++ ) {
		maxArray[i].begin(MAX31865_4WIRE);
	}

	// Setup PID objects
	for (int i = 0; i < pwmN; i++ ) {
		pidArray[i].SetMode(AUTOMATIC);
		pidArray[i].SetOutputLimits(0,4095);
		pidArray[i].SetTunings(consKp, consKi, consKd);
	}

	/* Set pinmodes */
	pinMode(PIN_DC_SENSOR, INPUT);
	for(int i = 0; i < pwmN; i++ ){
		pinMode(pwmPinArray[i], OUTPUT);
	}
}


/********/
/* Loop */
/********/
void loop() {

	// // TODO: Change parameters after heating up
	for (int i = 0; i < pwmN; i++ ) {
		pidArray[i].SetTunings(consKp, consKi, consKd);
	}

	// checkSerial();

	// checkInstruction();

	// checkVoltage();

	// For every PT100 read the values and check errors, then calculate the PWM
	for (int i = 0; i < maxN; i++ ) {
		//  Send MAX31865 number to target
		sendMaxNumber(i);

		// Read temperature values
		tempArray[i] = maxArray[i].temperature(resitanceRTD, resitanceReference);

		// Send data to target
		sendTempValues(i);

		// Check for errors
		checkFault(i);

		// Calculate PWM values
		calculatePWM(i);

		// Write PWM values
		analogWrite(pwmPinArray[i], pwmOutputArray[i]);

		// calculatePower();

		delay(10);
	}
	Serial.println();

	// TODO: replace with timer
	delay(1000);
}


/*************/
/* Functions */
/*************/
void calculatePWM(int i){
	// Take avergae for center MAX31865
	pidInputArray[i] = tempArray[i];
}

void calculatePower(){
	// Something
}

void checkVoltage(){
	// TODO: High frequency noice, if only checked irregularly than filter is required
	analogRaw = analogRead(PIN_DC_SENSOR);
	voltageIn = (analogRaw * voltageReference) / analogResolution;
	voltageSourceRaw = voltageIn / (resistorSmall/(resistorBig+resistorSmall));
	// Error compensation, check documentation for calibration data
	voltageSourceCompensated -= (0.0483*voltageSourceRaw)+0.0569;

	// //TODO: test
	// Serial.print("INPUT V= ");
 //  	Serial.println(voltageSourceCompensated, 2);
}

void checkFault(int i){
	faultArray[i] = maxArray[i].readFault();
	if (faultArray[i]) {

		// Print Error to target
		Serial.print('E');
		Serial.print('\n');
		Serial.print(faultArray[i], HEX);
		Serial.print('\n');

		// // Test print
		// Serial.print("("); Serial.print("Fault 0x"); Serial.print(faultArray[i], HEX); Serial.print(") ");

		// //TODO: in target
		// #define MAX31865_FAULT_HIGHTHRESH   0x80
		// #define MAX31865_FAULT_LOWTHRESH    0x40
		// #define MAX31865_FAULT_REFINLOW     0x20
		// #define MAX31865_FAULT_REFINHIGH    0x10
		// #define MAX31865_FAULT_RTDINLOW     0x08
		// #define MAX31865_FAULT_OVUV   		0x04


		// // TODO: receive error at target and check error value at target, thus eliminating this code
		// if (faultArray[i] & MAX31865_FAULT_HIGHTHRESH) {
		// 	Serial.print("RTD High Threshold");
		// }
		// if (faultArray[i] & MAX31865_FAULT_LOWTHRESH) {
		// 	Serial.print("RTD Low Threshold");
		// }
		// if (faultArray[i] & MAX31865_FAULT_REFINLOW) {
		// 	Serial.print("REFIN- > 0.85 x Bias");
		// }
		// if (faultArray[i] & MAX31865_FAULT_REFINHIGH) {
		// 	Serial.print("REFIN- < 0.85 x Bias - FORCE- open");
		// }
		// if (faultArray[i] & MAX31865_FAULT_RTDINLOW) {
		// 	Serial.print("RTDIN- < 0.85 x Bias - FORCE- open");
		// }
		// if (faultArray[i] & MAX31865_FAULT_OVUV) {
		// 	Serial.print("Under/Over voltage");
		// }

		maxArray[i].clearFault();
	}
}

// Send MAX31865 number over serial connection to target
void sendMaxNumber(int i){
	Serial.print('D');
	Serial.print('\n');
	if(i >= 5)	Serial.print((PIN_MAX_BOTTOM - 11) + (i * 2));
	else Serial.print(PIN_MAX_BOTTOM + (i * 2));
	Serial.print('\n');
}

// Send temperature values over serial connection to target
void sendTempValues(int i){
	Serial.print('V');
	Serial.print('\n');
	Serial.print(tempArray[i]);
	Serial.print('\n');
}

/* Serial */
// Check incoming serial data
void checkSerial(){
	//Read signal from computer
	if (Serial.available()){
		// Read the incoming byte
		incomingByte = Serial.read();
		// Echo incoming Byte
		Serial.write(incomingByte);
	}
}

void checkInstruction(){
	if(incomingByte == 'S') {
		// set up
	}
	if(incomingByte == 'P') {
		// prepare
	}
	if(incomingByte == 'R') {
		// run test
	}
	if(incomingByte == 'C') {
		// Cancel
	}
}
