/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInOutSerial
*/

// These constants won't change. They're used to give names to the pins used:
const int analogInPin0 = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin1 = A1;

int sensorValue1 = 0;
int sensorValue2 = 0;
unsigned long mt = 0;

void setup() {
  
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Serial.print("  t;");
  Serial.print("   A0; ");
  Serial.print("A1; ");
  Serial.println();
  
}


unsigned long t=0;
void loop() {

  if((millis()-t)>20000){
    //read the analog in value:
    sensorValue1 = analogRead(analogInPin0);
    sensorValue2 = analogRead(analogInPin1);
    mt=millis();
    t+=20000;
    Serial.print(mt);
    Serial.print("; ");
    // print the results to the Serial Monitor:
    Serial.print(sensorValue1);
    Serial.print("; ");
    Serial.print(sensorValue2);
    Serial.print("; ");
    Serial.println();
  }
}
