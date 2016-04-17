/*
 * PID Controller Implementation in C for Arduino - Usage example
 * 
 * This example shows how to use the library to calculate error correction according to PID algorithm.
 */

#include "pid.h"


// initialise blank PID_Calibration struct and blank PID_State struct
PID_Calibration calibration;
PID_State state;


void setup() {
    // configure the calibration and state structs
    // dummy gain values
    calibration.kp = 1.0;
    calibration.ki = 1.0;
    calibration.kd = 1.0;
    // an initial blank starting state
    state.actual = 0.0;
    state.target = 0.0;
    state.time_delta = 1.0; // assume an arbitrary time interval of 1.0
    state.previous_error = 0.0;
    state.integral = 0.0;
    // start the serial line at 9600 baud!
    Serial.begin(9600);
}


void loop() {
    // read in two bytes from serial, assume first is the target value and
    // second is the actual value. Output calculated result on serial
    if (Serial.available() >= 2) {
        // retrieve one byte as target value, cast to double and store in state struct
        state.target = (double) Serial.read();
        // same as above for actual value
        state.actual = (double) Serial.read();
        // now do PID calculation and assign output back to state
        state = pid_iterate(calibration, state);
        // print results back on serial
        Serial.print("Target:\t");
        Serial.println(state.target);
        Serial.print("Actual:\t");
        Serial.println(state.actual);
        Serial.print("Output:\t");
        Serial.println(state.output);
    }
}
