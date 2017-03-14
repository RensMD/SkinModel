/*
Copyright (c) 2016 TU Delft All Rights Reserved.
Rens Doornbusch
*/

// TODO: serial print(F()) vs Serial.print() vs Serial.write()
// TODO: Byte values working?
// TODO: Decrease globals

/**********/
/* Header */
/**********/
/* Libraries */
#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_MAX31865.h>
#include <PID_v1.h>

/* Constants */
// Pin layout
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

// Timer Cases
#define SEND					0
#define CONTROL				1

// Switch Cases
#define PREPERATION			0
#define TESTING				1
#define FINISHED				2
#define CANCELLED				3


// Resistance of MAX31865's Reference resitor and RTD at 0 degrees
const float resitanceReference	= 430.0;
const int resitanceRTD				= 100;

// Voltmeter
const float voltageReference = 2.50, resistorBig = 19820.0, resistorSmall = 938.0, analogResolution = 4095.0;

// Number of PWM and MAX31865 devices
const byte pwmN = 6, maxN = 10;

// PID
const float pidSwitchError = 1.5;
const float aggKp = 2000, aggKi = 0, aggKd = 0;
const float consKp = 1500, consKi = 0.01, consKd = 0;

/* Variables */
// Switch
byte state = PREPERATION;

// Serial
byte incomingByte;

// Voltmeter
float voltageSourceCompensated = 0.0;

// Timer
unsigned long timeControlPrevious = 0, timeSendPrevious = 0;
unsigned long timeControlInterval = 10000, timeSendInterval = 2000;

bool receivedStart 		= false;
bool receivedFinish 		= false;
bool receivedPrepare 	= false;
bool receivedCancel 		= false;
bool receivedTempTarget = false;

bool reachedTempTarget = false;

// PID + PWM
double tempTarget = 34.0;
double pidInput1; double pidInput2; double pidInput3; double pidInput4; double pidInput5; double pidInput6;
double pwmOutput1; double pwmOutput2; double pwmOutput3; double pwmOutput4; double pwmOutput5; double pwmOutput6;

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

/* Arrays */
// MAX31865
Adafruit_MAX31865 maxArray [maxN] = {maxBottom, maxSide1, maxSide2, maxSide3, maxSide4, maxCenter1, maxCenter2, maxCenter3, maxCenter4, maxCenter5};
byte maxPinArray [maxN] = {PIN_MAX_BOTTOM, PIN_MAX_SIDE1, PIN_MAX_SIDE2, PIN_MAX_SIDE3, PIN_MAX_SIDE4, PIN_MAX_CENTER1, PIN_MAX_CENTER2, PIN_MAX_CENTER3, PIN_MAX_CENTER4, PIN_MAX_CENTER5};
float tempArray [maxN] = {temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10};
uint16_t faultArray [maxN] = {fault1, fault2, fault3, fault4, fault5, fault6, fault7, fault8, fault9, fault10};

// PWM
PID pidArray [pwmN] = {myPID1, myPID2, myPID3, myPID4, myPID5, myPID6};
byte pwmPinArray [pwmN] = {PIN_PWM_BOTTOM, PIN_PWM_SIDE1, PIN_PWM_SIDE2, PIN_PWM_SIDE3, PIN_PWM_SIDE4, PIN_PWM_CENTER};
double pidInputArray [pwmN] = {pidInput1, pidInput2, pidInput3, pidInput4, pidInput5, pidInput6};
double pwmOutputArray [pwmN] = {pwmOutput1, pwmOutput2, pwmOutput3, pwmOutput4, pwmOutput5, pwmOutput6};


/*********/
/* Setup */
/*********/
void setup() {
	// Set resolution to 12bit (0-4095)
	analogWriteResolution(12);
	analogReadResolution(12);

	// TODO: Serial.begin was 19200! change to faster?
	Serial.begin(19200);

	// Setup MAX31865 objects
	for (byte i = 0; i < maxN; i++ ){
		maxArray[i].begin(MAX31865_4WIRE);
	}

	// Setup PID objects
	for (byte i = 0; i < pwmN; i++ ){
		pidArray[i].SetMode(AUTOMATIC);
		pidArray[i].SetOutputLimits(0,4095);
		pidArray[i].SetTunings(aggKp, aggKi, aggKd);
	}

	/* Set pinmodes */
	pinMode(PIN_DC_SENSOR, INPUT);
	for(byte i = 0; i < pwmN; i++ ){
		pinMode(pwmPinArray[i], OUTPUT);
	}

	for (byte m = 0; m < maxN; m++ ){
		if(checkFault(m)){
			sendFault(m);
			// TODO: let op, dit hier?
			state = CANCELLED;
		}
	}
}


/********/
/* Loop */
/********/
void loop() {
	if(checkSerial()){
		getInstruction();
	}

	switch(state){
		case PREPERATION:
			receivedPrepare = false;

			if(receivedCancel){
				state = CANCELLED;
			}
			else if(receivedTempTarget){
				if(reachedTempTarget){
					Serial.println("Setpoint reached");
					Serial.println("Press Start");
					if(receivedStart){
						state = TESTING;
					}
				}
			}

			// TODO: check reachedTempTarget

			if(checkTimer(CONTROL)){
				for (byte m = 0; m < maxN; m++ ){
					tempArray[m] = maxArray[m].temperature(resitanceRTD, resitanceReference);
					//  Send MAX31865 number to target
					sendMax(m);
					// Send data to target
					sendTemp(m);
				}
				for (byte p = 0; p < pwmN; p++ ){
					// Calculate PWM values
					calculatePwm(p);
					// Write PWM values
					analogWrite(pwmPinArray[p], pwmOutputArray[p]);
				}
			}
		break;

		case TESTING:
			receivedStart = false;

			if(checkTimer(SEND)){
				// For every MAX31865 check for errors
				for (byte m = 0; m < maxN; m++ ){
					if(checkFault(m)){
						sendFault(m);
						state = CANCELLED;
					}
				}

				// For every MAX31865 read the value
				for (byte m = 0; m < maxN; m++ ){
					tempArray[m] = maxArray[m].temperature(resitanceRTD, resitanceReference);
					delay(1);
				}

				// For every PWM Controller calculate and write values
				if(checkTimer(CONTROL)){
					for (byte p = 0; p < pwmN; p++ ){
						// Calculate PWM values
						calculatePwm(p);
						// Write PWM values
						analogWrite(pwmPinArray[p], pwmOutputArray[p]);
					}
				}

				// Send MAX31865 data
				for (byte m = 0; m < maxN; m++ ){
					//  Send MAX31865 number to target
					sendMax(m);
					// Send data to target
					sendTemp(m);
				}

				// Send PWM data
				for (byte p = 0; p < pwmN; p++ ){
					// Send PWM number
					sendPwm(p);
					// Send Output
					sendOutput(p);
				}

				// Send Voltage to target
				checkVoltage();
				sendVoltage();
			}
			Serial.println();
		break;

		case FINISHED:
			receivedFinish = false;
		break;

		case CANCELLED:
			// TODO: RUN ONCE in if
			if(receivedCancel){
				Serial.print('A');
			}
			receivedCancel = false;
		break;
	}
}


/*************/
/* Functions */
/*************/
bool checkTimer(byte t){
	if(t){
		if(millis() - timeControlPrevious > timeControlInterval){
			timeControlPrevious = millis();
			return true;
		}
	}
	else{
		if(millis() - timeSendPrevious > timeSendInterval){
			timeSendPrevious = millis();
			return true;
		}
	}
	return false;
}

bool checkFault(byte i){
	faultArray[i] = maxArray[i].readFault();
	if (faultArray[i]){
		return true;
	}
	maxArray[i].clearFault();
	return false;
}

void calculatePwm(byte i){
	if(i <= 4){
		pidInputArray[i] = tempArray[i];
		checkPidSwitch(tempArray[i], i);
		pidArray[i].Compute();
	}

	// Take average for the five center MAX31865's
	else if(i == 5){
		float inputTotal = 0;
		for(byte m = 5; m <= 9; m++){
			inputTotal += tempArray[m];
		}
		float inputAverage = inputTotal / 5;

		pidInputArray[i] = inputAverage;
		checkPidSwitch(inputAverage, i);
		pidArray[i].Compute();
	}
}

void checkPidSwitch(float tempInput, byte i){
	// When Error is bigger than defined PID Switch error, use aggresive tuning
	if((tempTarget - tempInput) > pidSwitchError){
		pidArray[i].SetTunings(aggKp, aggKi, aggKd);
	}
	else{
		pidArray[i].SetTunings(consKp, consKi, consKd);
	}
}

void checkVoltage(){
	// TODO: High frequency noice, if only checked irregularly than filter might be required
	float analogRaw = analogRead(PIN_DC_SENSOR);
	float voltageIn = (analogRaw * voltageReference) / analogResolution;
	float voltageSourceRaw = voltageIn / (resistorSmall/(resistorBig+resistorSmall));
	// Error compensation, check the documentation for info on the calibration constants
	voltageSourceCompensated -= (0.0483*voltageSourceRaw)+0.0569;
}

/* Serial */
// Check incoming serial data
bool checkSerial(){
	//Read signal from computer
	if (Serial.available()){
		// Read the incoming byte
		incomingByte = Serial.read();
		// Echo incoming Byte
		Serial.write(incomingByte);
		return true;
	}
	return false;
}

void getInstruction(){
	switch(incomingByte){
		case 'C':
			receivedCancel = true;
		break;

		case 'S':
			receivedStart = true;
		break;

		case 'F':
			receivedFinish = true;
		break;

		case 'P':
			receivedPrepare = true;
		break;

		case 'T':
		// TODO: LET OP already echoing byte
			Serial.print('K');
			Serial.print('\n');
			tempTarget = Serial.read();
			// TODO: ProcessIncomingData()
			receivedTempTarget = true;
		break;
	}
	incomingByte=' ';
}

// Send MAX31865 data
// Send MAX31865 number over serial connection to target
void sendMax(byte i){
	Serial.print('M');
	Serial.print('\n');
	Serial.print(maxPinArray[i]);
	Serial.print('\n');
}

// Send temperature values over serial connection to target
void sendTemp(byte i){
	Serial.print('T');
	Serial.print('\n');
	Serial.print(tempArray[i]);
	Serial.print('\n');
}

// Send fault  over serial connection to target
void sendFault(byte i){
	Serial.print('F');
	Serial.print('\n');
	Serial.print(faultArray[i], HEX);
	Serial.print('\n');
}

// Send PWM data
// Send PWM number over serial connection to target
void sendPwm(byte i){
	Serial.print('P');
	Serial.print('\n');
	Serial.print(pwmPinArray[i]);
	Serial.print('\n');
}

// Send voltage over serial connection to target
void sendOutput(byte i){
	Serial.print('V');
	Serial.print('\n');
	Serial.print(pwmOutputArray[i]);
	Serial.print('\n');
}

// Send voltage over serial connection to target
void sendVoltage(){
	Serial.print('V');
	Serial.print('\n');
	Serial.print(voltageSourceCompensated);
	Serial.print('\n');
}
