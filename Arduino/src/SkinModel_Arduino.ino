/*
Copyright (c) 2016 TU Delft All Rights Reserved.
Developped by: Rens Doornbusch
*/

// TODO: serial print(F()) vs Serial.println() vs Serial.write()
// TODO: Explanatory Header

/**********/
/* Header */
/**********/
/* Libraries */
#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_MAX31865.h>

/* Constants */
// Pin layout //
// Voltmeter
#define PIN_DC_SENSOR		A0

// PWM Heater pins
#define PIN_PWM_BOTTOM		2
#define PIN_PWM_SIDE1		3
#define PIN_PWM_SIDE2		4
#define PIN_PWM_SIDE3		5
#define PIN_PWM_SIDE4		6
#define PIN_PWM_CENTER		7

// MAX31865 or PT100 pins
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

// Cases //
// Timer Cases
#define SEND									0
#define CONTROL								1
#define ERROR_CHECK							2

// Cancel Cases
#define NONE									0
#define ERROR_SETUP							1
#define ERROR_LOOP_PREPERATION			2
#define ERROR_LOOP_TESTING					3
#define CANCEL_RECEIVED_PREPERATION		4
#define CANCEL_RECEIVED_TESTING			5

// Switch Cases
#define PREPERATION							0
#define TESTING								1
#define FINISHED								2
#define CANCELLED								3

// Values //
// Resistance of MAX31865's Reference resitor and RTD at 0 degrees
const float resitanceReference = 430.0;
const int resitanceRTD = 100;

// Resistance of Heaters
const float resistancePWMBottom = 41.6, resistancePWMSide1 = 347.2, resistancePWMSide2 = 407.1, resistancePWMSide3 = 345.3, resistancePWMSide4 = 407.7, resistancePWMCenter = 72.7;

// Voltmeter
const float voltageReference = 2.50, resistorBig = 19820.0, resistorSmall = 938.0, analogResolution = 4095.0;

// Number of PWM Heaters and MAX31865 devices
const byte pwmN = 6, maxN = 10;

/* Variables */
// Cases //
byte cancelReason = NONE;

byte state = PREPERATION;

// Serial //
int incomingByte = ' ';

// Flags //
// Flags, triggered by Serial input
bool receivedStart 					= false;
bool receivedFinish 					= false;
bool receivedCancel 					= false;
bool receivedRestart				 	= false;

bool receivedIgnoreFault			= false;

bool receivedTempTarget 			= false;
bool receivedAllowedDeviation		= false;

// Flags, triggered by event
bool reachedTempTarget 				= false;

// Values //
// Target Temperature
float tempTarget = 34.0;
float allowedDeviation = 0.1;

// Timer
unsigned long timeControlPrevious = 0, timeSendPrevious = 0, timeErrorCheckPrevious = 0;
unsigned long timeControlInterval = 1000, timeSendInterval = 2000, timeErrorCheckInterval = 30000;
unsigned long timeStart, timePrevious, timeNow;

// Voltmeter
float voltageEffectiveBottom, voltageEffectiveSide1, voltageEffectiveSide2, voltageEffectiveSide3, voltageEffectiveSide4, voltageEffectiveCenter;

// Energy
float energyBottom, energySide1, energySide2, energySide3, energySide4, energyCenter;

// PWM
float pwmOutput1, pwmOutput2, pwmOutput3, pwmOutput4, pwmOutput5, pwmOutput6;

// MAX31865 variables
float temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10;
uint16_t fault1, fault2, fault3, fault4, fault5, fault6, fault7, fault8, fault9, fault10;

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

/* Arrays */
// Voltage
float voltageEffectiveArray [pwmN] = {voltageEffectiveBottom, voltageEffectiveSide1, voltageEffectiveSide2, voltageEffectiveSide3, voltageEffectiveSide4, voltageEffectiveCenter};
// Energy
float energyConsumptionArray [pwmN] = {energyBottom, energySide1, energySide2, energySide3, energySide4, energyCenter};

// MAX31865
Adafruit_MAX31865 maxArray [maxN] = {maxBottom, maxSide1, maxSide2, maxSide3, maxSide4, maxCenter1, maxCenter2, maxCenter3, maxCenter4, maxCenter5};
byte maxPinArray [maxN] = {PIN_MAX_BOTTOM, PIN_MAX_SIDE1, PIN_MAX_SIDE2, PIN_MAX_SIDE3, PIN_MAX_SIDE4, PIN_MAX_CENTER1, PIN_MAX_CENTER2, PIN_MAX_CENTER3, PIN_MAX_CENTER4, PIN_MAX_CENTER5};
float tempArray [maxN] = {temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10};
uint16_t faultArray [maxN] = {fault1, fault2, fault3, fault4, fault5, fault6, fault7, fault8, fault9, fault10};

// PWM Heaters
byte pwmPinArray [pwmN] = {PIN_PWM_BOTTOM, PIN_PWM_SIDE1, PIN_PWM_SIDE2, PIN_PWM_SIDE3, PIN_PWM_SIDE4, PIN_PWM_CENTER};
float pwmOutputArray [pwmN] = {pwmOutput1, pwmOutput2, pwmOutput3, pwmOutput4, pwmOutput5, pwmOutput6};
float pwmResistanceArray [pwmN] = {resistancePWMBottom, resistancePWMSide1, resistancePWMSide2, resistancePWMSide3, resistancePWMSide4, resistancePWMCenter};


/*********/
/* Setup */
/*********/
void setup() {
	/* Settings */
	// Set resolution to 12bit (0-4095)
	analogWriteResolution(12);
	analogReadResolution(12);

	/* Setup Objects */
	// TODO: Change to faster serial communication?
	Serial.begin(19200);

	// Setup MAX31865 objects
	for (byte i = 0; i < maxN; i++ ){
		maxArray[i].begin(MAX31865_4WIRE);
	}

	/* Set pinmodes */
	// Setup PWM Heater pins
	for(byte i = 0; i < pwmN; i++ ){
		pinMode(pwmPinArray[i], OUTPUT);
	}
	// Setup Voltmeter pin
	pinMode(PIN_DC_SENSOR, INPUT);

	/*  Initial Checks */
	// Check for faults in MAX31865's
	if(checkFault()){
		state = CANCELLED;
		cancelReason = ERROR_SETUP;
	}
}


/********/
/* Loop */
/********/
void loop() {
	delay(1);

	/* Check Serial */
	if(checkSerial()){
		getSerialInstruction();
	}

	/* State Machine */
	switch(state){
		case PREPERATION:
			/* State Transitions */
			// Run once:
			if(receivedRestart){
				receivedRestart = false;
				receivedTempTarget = false;
				receivedAllowedDeviation = false;
			}

			if(receivedCancel){
				state = CANCELLED;
				cancelReason = CANCEL_RECEIVED_PREPERATION;
				break;
			}

			if(receivedTempTarget && receivedAllowedDeviation && reachedTempTarget){
				if(receivedStart){
					state = TESTING;
					break;
				}
				if(checkTimer(SEND)){
					// Serial.println("Reached Temp Target, Press Start");
					Serial.println('R');
				}
			}

			// Check for MAX31865 errors
			if(checkTimer(ERROR_CHECK)){
				if(checkFault()){
					cancelReason = ERROR_LOOP_PREPERATION;
					state = CANCELLED;
					break;
				}
			}

			/* State Actions */
			if(checkTimer(CONTROL)){
				for (byte m = 0; m < maxN; m++ ){
					// Read MAX31865 temperatures
					tempArray[m] = maxArray[m].temperature(resitanceRTD, resitanceReference);
					delay(1);

					if(checkTimer(SEND)){
						//  Send MAX31865 number to target
						sendMax(m);
						// Send data to target
						sendTemp(m);
					}
				}
				for (byte p = 0; p < pwmN; p++ ){
					// Calculate PWM values
					calculatePWM(p);
					// Write PWM values
					analogWrite(pwmPinArray[p], pwmOutputArray[p]);
				}
			}
		break;

		case TESTING:
			/* State Transitions */
			//Run once:
			if(receivedStart){
				receivedStart = false;
				// Reset values for start
				timeStart = millis();
				timePrevious = timeStart;
				for (byte p = 0; p < pwmN; p++ ){
					energyConsumptionArray[p] = 0;
				}
			}

			if(receivedCancel){
				state = CANCELLED;
				cancelReason = CANCEL_RECEIVED_TESTING;
				break;
			}

			if(receivedFinish){
				state = FINISHED;
				break;
			}

			// Check for MAX31865 errors
			if(checkTimer(ERROR_CHECK)){
				if(checkFault()){
					cancelReason = ERROR_LOOP_TESTING;
					state = CANCELLED;
					break;
				}
			}

			/* State Actions */
			if(checkTimer(CONTROL)){
				// For every MAX31865 read the value
				for (byte m = 0; m < maxN; m++ ){
					tempArray[m] = maxArray[m].temperature(resitanceRTD, resitanceReference);
					delay(1);
				}

				// For every PWM Controller calculate and write values
				for (byte p = 0; p < pwmN; p++ ){
					// Calculate PWM values
					calculatePWM(p);
					// Write PWM values
					analogWrite(pwmPinArray[p], pwmOutputArray[p]);
				}

				calculateVoltage();
				calculateEnergy();
			}

			if(checkTimer(SEND)){
				// Send MAX31865 data
				for (byte m = 0; m < maxN; m++ ){
					sendMax(m);
					sendTemp(m);
				}

				// Send PWM Heater data
				for (byte p = 0; p < pwmN; p++ ){
					sendPWM(p);
				}
			}
		break;

		case FINISHED:
			/* State Transitions */
			// Run once:
			if(receivedFinish){
				// Serial.println("Done");
				Serial.println('D');
				receivedFinish = false;
			}

			if(receivedRestart){
				state = PREPERATION;
				break;
			}
		break;

		case CANCELLED:
			/* State Transitions */
			// Run once:
			if(receivedCancel){
				// Serial.println("Cancelled: ");
				Serial.println('C');
				Serial.println(cancelReason);
				receivedCancel = false;
			}

			if(receivedRestart){
				state = PREPERATION;
				break;
			}
		break;
	}
}


/*************/
/* Functions */
/*************/
/* Other */
// Timer //
// Check if enough time has passed
bool checkTimer(byte timeCheck){
	switch(timeCheck){
		case CONTROL:
			if(millis() - timeControlPrevious > timeControlInterval || timeControlPrevious==0){
				timeControlPrevious = millis();
				return true;
			}
			else
				return false;
		break;

		case SEND:
			if(millis() - timeSendPrevious > timeSendInterval){
				timeSendPrevious = millis();
				return true;
			}
			else
				return false;
		break;

		case ERROR_CHECK:
			if(millis() - timeErrorCheckPrevious > timeErrorCheckInterval){
				timeErrorCheckPrevious = millis();
				return true;
			}
			else
				return false;
		break;
	}
}

// Errors //
// Check for Faults in MAX31865
bool checkFault(){
	bool fault = false;
	for (byte m = 0; m < maxN; m++){
		faultArray[m] = maxArray[m].readFault();
		maxArray[m].clearFault();
		if (faultArray[m]){
			sendMax(m);
			sendFault(m);
			fault = true;
		}
	}
	if(fault)
		if(!receivedIgnoreFault)
			return true;
	else
		return false;
}

// Map Float //
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max){
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

/* Control */
// TODO: The current controller is a dummy controller,
// TODO: Build a controller for the system, which is resilient to dynamic changes
// Calculate the Output PWM
void calculatePWM(byte i){
	if(i <= 4){
		if(tempArray[i]>tempTarget)
			pwmOutputArray[i]--;
		else if(tempArray[i]<tempTarget)
			pwmOutputArray[i]++;
	}

	// Take average for the five center MAX31865's
	else if(i == 5){
		// TODO: Selection option for which PT100's to use for Calculation
		// Example:
		// use sensor 6 and 8
		// take tempArray[6] and tempArray[8]
		// use those to calculate average
		// use average for control

		float inputTotal = 0;
		for(byte m = 5; m <= 9; m++)
			inputTotal += tempArray[m];
		float inputAverage = inputTotal / 5;

		if(inputAverage > tempTarget + allowedDeviation){
			pwmOutputArray[i]--;
			reachedTempTarget = false;
		}
		else if(inputAverage < tempTarget - allowedDeviation){
			pwmOutputArray[i]++;
			reachedTempTarget = false;
		}
		else if(inputAverage<=tempTarget+allowedDeviation && inputAverage>tempTarget-allowedDeviation)
			reachedTempTarget = true;
	}

	// Set the time on the moment of control change
	timePrevious = timeNow;
	timeNow = millis();
}

// Measurements //
// Calculate the current Voltage
// TODO: Found high frequency noise during tests! If only checked with low frequency than filter might be required
void calculateVoltage(){
	float analogRaw = analogRead(PIN_DC_SENSOR);
	float voltageIn = (analogRaw * voltageReference) / analogResolution;
	float voltageSourceRaw = voltageIn / (resistorSmall/(resistorBig+resistorSmall));
	// Error compensation, check the documentation for info on the calibration constants
	float voltageSourceCompensated = voltageSourceRaw - (0.0483 * voltageSourceRaw) + 0.0569;
	// For voltageLost data check documentation MOSFET Voltagelost
	float voltageLost = 0;
	for (byte p = 0; p < pwmN; p++ ){
		if(p==0)
			voltageLost = 0.2;
		else if(p==1)
			voltageLost = voltageSourceCompensated / 5000;
		else if(p==2)
			voltageLost = 0.0003 * (voltageSourceCompensated * voltageSourceCompensated) - (0.0071 * voltageSourceCompensated) + 0.0804;
		else if(p==3)
			voltageLost = voltageSourceCompensated / 7500;
		else if(p==4)
			voltageLost = voltageSourceCompensated / 10000;
		else if(p==5)
			voltageLost = 0.0027 * voltageSourceCompensated - 0.0038;

		voltageEffectiveArray[p] = voltageSourceCompensated - voltageLost;
	}
}

void calculateEnergy(){
	for (byte p = 0; p < pwmN; p++ ){
		float voltageLoad = voltageEffectiveArray[p] * mapfloat(pwmOutputArray[p], 0, 4095, 0, 1);
		// Power Calculation P=(V^2)/R
		float power = (voltageLoad * voltageLoad) / pwmResistanceArray[p];
		// Energy Calculation E=P*t
		energyConsumptionArray[p] += power * ((timeNow-timePrevious)/1000);
	}
}

/* Serial */
// Receive Data //
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

// Check incoming data for instructions
void getSerialInstruction(){
	switch(incomingByte){
		case 'S':
			receivedStart = true;
		break;

		case 'F':
			receivedFinish = true;
		break;

		case 'C':
			receivedCancel = true;
		break;

		case 'R':
			receivedRestart = true;
		break;

		case 'I':
			receivedIgnoreFault = true;
		break;

		case 'T':
			tempTarget = getSerialValue();
			receivedTempTarget = true;
		break;

		case 'D':
			allowedDeviation = getSerialValue();
			receivedAllowedDeviation = true;
		break;
	}
	incomingByte=' ';
}

// Check incoming data for usable values
// TODO: Check if function is working
// TODO: Which type for return function?
// TODO: Create fitting error if necessary
float getSerialValue(){
	int w = 0;
	while(!Serial.available() && w < 10){
		w++;
		Serial.write('T');
		delay(10);
	}
	delay(100);
	if(Serial.available()){
		Serial.write('#');
		return Serial.read();
	}
	else
		Serial.write("Can't receive Data!");
}

// Send Data //
// Send MAX31865 data
// Send MAX31865 number over serial connection to target
void sendMax(byte i){
	Serial.println('M');
	Serial.println(maxPinArray[i]);
}

// Send temperature values over serial connection to target
void sendTemp(byte i){
	Serial.println('T');
	Serial.println(tempArray[i]);
}

// Send fault  over serial connection to target
void sendFault(byte i){
	Serial.println('F');
	Serial.println(faultArray[i], HEX);
}

// Send PWM data
// Send PWM Heater number, output and energy consumption over serial connection to target
void sendPWM(byte i){
	Serial.println('P');
	Serial.println(pwmPinArray[i]);
	Serial.println('O');
	Serial.println(pwmOutputArray[i]);
	Serial.println('E');
	Serial.println(energyConsumptionArray[i]);
	Serial.println('V');
	Serial.println(voltageEffectiveArray[i]);
	Serial.println('S');
	Serial.println(timeNow);
}
