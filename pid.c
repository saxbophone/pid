/*
 * PID Controller Implementation in C
 * 
 * Created by Joshua Saxby (aka @saxbophone) on 1 Jan, 2016
 * 
 * My own attempt at implementing the PID algorithm in some (hopefully) clean, understandable C.
 *
 * See LICENSE for licensing details.
 */

#include "pid.h"


PID_State pid_iterate(PID_Calibration calibration, PID_State state) {
    // calculate difference between desired and actual values (the error)
    double error = state.target - state.actual;
    // calculate and update integral
    state.integral += (error * state.time_delta);
    // calculate derivative
    double derivative = (error - state.previous_error) / state.time_delta;
    // calculate output value according to algorithm
    state.output = (
        (calibration.kp * error) + (calibration.ki * state.integral) + (calibration.kd * derivative)
    );
    // update state.previous_error to the error value calculated on this iteration
    state.previous_error = error;
    // return the state struct reflecting the calculations
    return state;
}
