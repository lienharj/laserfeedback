#include <Indio.h>
#include <Wire.h>
#include "Sinewave.h"       // file with float sine values [-1,1] in array
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

const int arg_M = 100;
float arg_fc = 0.01;
float arg_coefs[arg_M + 1];
float data[arg_M + 1];
int start = 0;
int controller = 1;                       // 1 = MAF, 2 = BMF

float blackman_coefs(float* arg_coefs){   // calculate normalized coeffizients
  float sk = 0;                           // normalization constant
  int k = 0;
  for (k = 0; k <= arg_M; k++) {          // calculate coeffizients
    if (k != arg_M / 2.0) {
      arg_coefs[k] = sin(2 * M_PI * arg_fc * (k - arg_M / 2.0)) / (k - arg_M / 2.0) * (0.42 - 0.5 * cos(2 * M_PI * k / arg_M) + 0.08 * cos(4.0 * M_PI * k / arg_M));
      sk += arg_coefs[k];
    }
    else {
      arg_coefs[k] = 2 * M_PI * arg_fc;
      sk += arg_coefs[k];
    }
  }
  for (k = 0; k <= arg_M; k++) {          // normalization
    arg_coefs[k] /= sk;
  }
  return 0;
}

float blackman_filter( float* arg_raw_data, int start){   // BM filter
  float out = 0;
  int k = 0;
  int pos = 0;
  for (k = 0; k <= arg_M; k++) {
    pos = start - k;
    if (pos < 0) {
      pos += arg_M + 1;
    }
    out += arg_raw_data[pos] * arg_coefs[k];
  }
  return out;
}

float ma_filter( float* sensRawArray, int filterSamples, int start)   // MAF filter
{
  float sum = 0;
  int k = 0;
  int pos = 0;
  for (k = 0; k < filterSamples; k++) {
    pos = start - k;
    if (pos < 0) {
      pos += filterSamples;
    }
    sum += sensRawArray[pos];
  }
  sum /= filterSamples;
  return sum;
}

void setup() {
  //serial:
  SerialUSB.begin(230400);                   // opens serial port
  //prepare pins:
  pinMode(controller_pin, INPUT);
  Indio.setADCResolution(12);               // sets analog input resolution to 12bit
  Indio.analogReadMode(input_pin, V10);
  Indio.analogWriteMode(output_pin, V10);
  Indio.analogWrite(output_pin, 0, true);   // fail-save default output set to 0V
  Indio.analogWriteMode(analog_control_pin, V10);
  Indio.analogWrite(output_pin, 5, false);  // activate analog control

  if(controller == 2){
    blackman_coefs(arg_coefs);                // executes coefficents calculations
  }
  for (int k = 0; k <=arg_M; k++) {         // sets data aray to 0
    data[k] = 0;
  }
}

void loop() {
  if (j % 50 == 0) {                // every 50 loop another sine value is outputed
    controller_status = digitalRead(controller_pin);  // checks button state
    if (controller_status) {
      // SerialUSB.println(Laser on);
      output = sinewaveTable[i] * sine_amplitude + sine_vshift;
      if (output > 5.0) {                     // checks that output is not to high
        output = 5.0;
      }
      else if (output < 0) {
        output = 0.0;
      }
      i++;
      if (i == maxSamplesNum) {    // if sine samples are all through, set to 0 again
        i = 0;
      }
    }
    else {
      output = 0;
      i = 0;

    }
    Indio.analogWrite(output_pin, output, false);    //  outputs output to plant
  }
  data[start] = Indio.analogRead(in_pin);           // reads photodiodes value
  if (controller == 1) {
    input = ma_filter(data, arg_M + 1, start);              // filters with MAF
  }
  else if (controller == 2) {
    input = blackman_filter(data, start);             // filters with BM
  }
  serial_output=  String(micros(),DEC)+" "+(int)(data[start] * precision)+ " " + (int)(input * precision)+ " " + (int)(output * precision);
  SerialUSB.println(serial_output);                 //  outputs data to serial port
  j++;
  start++;
  if (start == arg_M+1) {
    start = 0;
  }
}
