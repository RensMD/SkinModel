/*
  DC Voltmeter Using a Voltage Divider
  + PWM Control MOSFETS
*/

#define analogInput A0 
#define pwmPin1 2
#define pwmPin2 3
#define pwmPin3 4
#define pwmPin4 5
#define pwmPin5 6
#define pwmPin6 7

float vout = 0.0;       // Calculated voltage 
float vin = 0.0;        // Resulting voltage
float R1 = 19820.0;     // Resistor 1  
float R2 = 938.0;       // Resistor 2
float raw = 0;          // Raw ADC measurement
float vref = 2.50;      // Reference voltage
//float res = 1023.0;     // Resolution of the ADC
float res = 4095.0;     // Resolution of the ADC

void setup(){
  pinMode(analogInput, INPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);
  pinMode(pwmPin5, OUTPUT);
  pinMode(pwmPin6, OUTPUT);

//  analogReference(EXTERNAL); 
  analogReadResolution(12);
//  analogWriteResolution(12);
   
  Serial.begin(9600);
}
void loop(){
  analogWrite(pwmPin1, 255);
  analogWrite(pwmPin2, 255);
  analogWrite(pwmPin3, 255);
  analogWrite(pwmPin4, 255);
  analogWrite(pwmPin5, 255);
  analogWrite(pwmPin6, 255);
  
  raw = analogRead(analogInput);
  vout = (raw * vref) / res; 
  vin = vout / (R2/(R1+R2));
  vin -= (0.0483*vin)+0.0569;
  
  Serial.print("INPUT V= ");
  Serial.println(vin,3);
  
  delay(100);
}
