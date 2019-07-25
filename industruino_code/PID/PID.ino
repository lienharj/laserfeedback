#include <Indio.h>
#include <Wire.h>
#include <PID_v1.h>                   // PID library

const int controller_pin = 7;  // digital pin to switch controller on/off via button
bool controller_status = false;
const int input_pin = 1;              // ADC CH1 pin
const int output_pin = 1;             // DAC CH1 pin
const int analog_control_pin = 2;     // DAC CH2 pin for activating analog control of plant

double kp = 5.0, ki = 0.3, kd = 0.0;  // PID parameters
double input = 0;                     // sensor value [0,5] (Volts)
double output = 0;                    // output for plant [0, 5]  (Volts)
double setpoint = 0.5;                // desired setpoint [0, 5]
double maxLim = 1.5;                  // max output limit
double minLim = 0.0;                  // min output limit
int sample_time = 3;                  // sample time in ms
int precision = 1000;
String serial_output;                 // variable to create one single output string

PID myPID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);

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
  // prepare PID:
  myPID.SetTunings(kp, ki, kd);
  myPID.SetOutputLimits(minLim,maxLim);
  myPID.SetSampleTime(sample_time);    // determines the sampling time of the PID algorithm
}

void loop() {
  controller_status = digitalRead(controller_pin);  // checks button state
  input = Indio.analogRead(input_pin);              // reads photodiodes value
  if (controller_status) {    // if botton is on, control is activated and computed
    myPID.SetMode(AUTOMATIC);
    myPID.Compute();
  }
  else {                      // if botton is off, control is deactivated
    output = 0;
    myPID.SetMode(MANUAL);
  }
  Indio.analogWrite(output_pin, output, false);     //  outputs output to plant
  serial_output =  String(micros(), DEC) + " " + (int)(input * precision) + " " + (int)(output * precision);
  SerialUSB.println(serial_output);                 //  outputs data to serial port
}
