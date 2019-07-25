#include <Indio.h>
#include <Wire.h>

const int controller_pin = 7;  // digital pin to switch controller on/off via button
bool controller_status = false;
const int input_pin = 1;              // ADC CH1 pin
const int output_pin = 1;             // DAC CH1 pin
const int analog_control_pin = 2;     // DAC CH2 pin for activating analog control of plant

double kp = 2.0, ki = 0.000001, kd = 0.0;  // PID parameters
double ka = 0.5, sat = 3.25;
double input = 0;                     // sensor value [0,5] (Volts)
double output = 0;                    // output for plant [0, 5]  (Volts)
double setpoint = 0.3;                // desired setpoint [0, 5]
double error, lastError = 0;
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double cumError, rateError;
int precision = 1000;
double limit = 3.25;                  // max output limit
String serial_output;

void computePID() {                  // computes PID output
  currentTime = micros();
  elapsedTime = (double)(currentTime - previousTime);
  error = setpoint - input;
  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;
  output = kp * error + ki * cumError + kd * rateError;

  if (output > sat) {
    cumError = cumError - ka * (output-sat);
    output = sat;
  }
  else if (cumError < -sat) {
    cumError = cumError - ka * (output + sat);
    output = -sat;
  }
  lastError = error;
  previousTime = currentTime;
}

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
  controller_status = digitalRead(controller_pin);  // checks button state
  input = Indio.analogRead(input_pin);     // reads photodiodes value
  if (controller_status) {   // if botton is on, control is activated and computed
    computePID();                         // computes PID output
    if (output > limit) {                 // checks for output be below limit
      output = limit;
    }
    if (output < 0) {
      output = 0.0;
    }
  }
  else {
    output = 0.0;
  }
  Indio.analogWrite(output_pin, output, false);   //  outputs output to plant
  serial_output =  String(micros(), DEC) + " " + (int)(input * precision) + " " + (int)(output * precision);
  SerialUSB.println(serial_output);               //  outputs data to serial port
}
