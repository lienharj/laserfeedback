#include <Indio.h>
#include <Wire.h>

const int in_pin = 1;               //ADC CH1 Pin
double input;                       // sensor value [0,5] (Volts)
int precision = 1000;
String serial_output;               // variable to create one single output string

void setup() {
  SerialUSB.begin(230400);          // opens serial port
  Indio.setADCResolution(12);       // sets analog input resolution to 12bit
  Indio.analogReadMode(in_pin, V10);
}

void loop() {
  input = Indio.analogRead(in_pin);   // reads photodiodes value
  serial_output = String(micros(),DEC)+" "+(int)(input * precision);
  SerialUSB.println(serial_output);   //  outputs data to serial port
}
