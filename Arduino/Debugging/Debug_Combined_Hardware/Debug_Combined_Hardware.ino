#include <Arduino.h>
#include <SPI.h>

#include <Adafruit_MAX31865.h>

#define analogInput A0 
#define pwmPin1 2
#define pwmPin2 3
#define pwmPin3 4
#define pwmPin4 5
#define pwmPin5 6
#define pwmPin6 7

// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 maxBottom = Adafruit_MAX31865(22);
Adafruit_MAX31865 maxSide1 = Adafruit_MAX31865(23);
Adafruit_MAX31865 maxSide2 = Adafruit_MAX31865(24);
Adafruit_MAX31865 maxSide3 = Adafruit_MAX31865(25);
Adafruit_MAX31865 maxSide4 = Adafruit_MAX31865(26);
Adafruit_MAX31865 maxCenter1 = Adafruit_MAX31865(27);
Adafruit_MAX31865 maxCenter2 = Adafruit_MAX31865(28);
Adafruit_MAX31865 maxCenter3 = Adafruit_MAX31865(29);
Adafruit_MAX31865 maxCenter4 = Adafruit_MAX31865(30);
Adafruit_MAX31865 maxCenter5 = Adafruit_MAX31865(31);

// The value of the Rref resistor. Use 430.0!
#define RREF 430.0

const int n_Max =  10;
Adafruit_MAX31865 maxNames [n_Max] = {maxBottom, maxSide1, maxSide2, maxSide3, maxSide4, maxCenter1, maxCenter2, maxCenter3, maxCenter4, maxCenter5};

float vout = 0.0;       // Calculated voltage 
float vin = 0.0;        // Resulting voltage
float R1 = 19820.0;     // Resistor 1  
float R2 = 938.0;       // Resistor 2
float raw = 0;          // Raw ADC measurement
float vref = 2.50;      // Reference voltage
float res = 4095.0;     // Resolution of the ADC


/////////////////////////////////////
// SETUP
////////
void setup() {
  Serial.begin(115200);
  Serial.println("MAX31865 + PWM + Voltmeter tests");

  analogReadResolution(12);
  analogWriteResolution(12);

  for (int i = 0; i < n_Max; ++i ) {
    maxNames[i].begin(MAX31865_4WIRE);
  }

  pinMode(analogInput, INPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);
  pinMode(pwmPin5, OUTPUT);
  pinMode(pwmPin6, OUTPUT);
}

/////////////////////////////////////
// Loop
///////
void loop() {
  // PWM
  analogWrite(pwmPin1, 255);
  analogWrite(pwmPin2, 255);
  analogWrite(pwmPin3, 255);
  analogWrite(pwmPin4, 255);
  analogWrite(pwmPin5, 255);
  analogWrite(pwmPin6, 255);

  // Voltmeter
  raw = analogRead(analogInput);
  vout = (raw * vref) / res; 
  vin = vout / (R2/(R1+R2));
  vin -= (0.0483*vin)+0.0569;
  
  Serial.print("INPUT V= ");
  Serial.println(vin,3);
  
  // MAX
  uint16_t rtd = maxBottom.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("RTD value: "); Serial.println(rtd);
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(maxBottom.temperature(100, RREF));

  uint8_t fault = maxBottom.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold");
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold");
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias");
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage");
    }
    maxBottom.clearFault();
  }
  Serial.println();
  delay(100);
}
