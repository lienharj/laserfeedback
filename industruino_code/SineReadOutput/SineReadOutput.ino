#include <Indio.h>
#include <Wire.h>
#include "Sinewave.h"       // file with flaot sine values [-1,1] in array 
                              // sinewaveTable and array length maxSamplesNum
const int controller_pin = 7;  // digital pin to switch controller on/off via button
bool laser_status = false;
const int input_pin = 1;              // ADC CH1 Pin
const int output_pin = 1;             // DAC CH1 pin
const int analog_control_pin = 2;     // DAC CH2 pin for activating analog control of plant

float input = 0;
float output = 0;
float sine_amplitude = 0.25;    // Sine amplitude: 1-->20W ... 3-->60W
float sine_vshift = 2.0;        // Vertical sine shift: 1-->20W ... 5-->100W
int i = 0;                        // max power = (sine amplitude + sine vshift)*20W
unsigned int j = 1;
int precision = 1000;
String serial_output;

void setup() {
  //serial:
  SerialUSB.begin(230400);                  // opens serial port
  //prepare pins:
  pinMode(controller_pin, INPUT);
  Indio.setADCResolution(12);               // sets analog input resolution to 12bit
  Indio.analogReadMode(input_pin, V10);
  Indio.analogWriteMode(output_pin, V10);
  Indio.analogWrite(output_pin, 0, true);   // fail-save default output set to 0V
  Indio.analogWriteMode(analog_control_pin, V10);
  Indio.analogWrite(output_pin, 5, false);  // activate analog control
}

void loop() {
  if(j % 50 == 0){         // every 50 loop another sine value is outputed
    laser_status = digitalRead(controller_pin);   // checks button state
    if (laser_status) {
      output = sinewaveTable[i] * sine_amplitude + sine_vshift;
      if(output>5.0){                      // checks that output is not to high
        output=5.0;
      }
      else if (output<0){
        output=0.0;
      }
      i++;
      if (i == maxSamplesNum) {
        i = 0;
      }
    }
    else{
      output=0;
      i=0;
    }
    Indio.analogWrite(output_pin, output, false);  //  outputs output to plant
  }
  input = Indio.analogRead(input_pin);            // reads photodiodes value
  serial_output=  String(micros(),DEC)+ " " + (int)(input * precision)+ " " + (int)(output * precision);
  SerialUSB.println(serial_output);               //  outputs data to serial port
  j++;
}
