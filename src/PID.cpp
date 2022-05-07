#include "main.h"


// constructor for the PID class sets base value of '0' for all variables
PID::PID() {
  target = 0;
  error = 0;
  past_error = 0;
  integral = 0;
  derivative = 0;
  kp = 0;
  ki = 0;
  kd = 0;
  max = 0;
  min = 0;
  integral_limit = 0;
}




// sets the value for the PID constants
void PID::set_constants(double p, double i, double d) {
  kp = p;
  ki = i;
  kd = d;
}




// sets the value for the PID constants
void PID::set_constants(PIDConstants pid_var) {
  kp = pid_var.kp;
  ki = pid_var.ki;
  kd = pid_var.kd;
  this->integral_limit = pid_var.int_limit;
}




// sets the value of the target, the max output, the minimum output, and the integral limit
void PID::set_variables(double target_input, double max_value, double min_value, double integral_lim) {
  target = target_input;
  max = max_value;
  min = min_value;
  // just in case you enter a negative integral limit
  integral_limit = fabs(integral_lim);
}




// sets the value of the target, the max output, the minimum output, and the integral limit
void PID::set_target(double target) {
  this->target = target;
}



void PID::set_error(double error) {
  this->error = error;
}



void PID::set_past_error(double past_error) {
  this->past_error = past_error;
}



// returns the result of the PID calculations
double PID::output(double current) {
  double out;
  // set the value of the past error before it gets updated
  past_error = error;
  // set the error value based on the difference between the target and the current value
  error = target - current;
  // check if the error is within the intergal limit and if it is, calculate the integral
  if (fabs(error) < integral_limit) {
    integral += error;
  }
  else {
    integral = 0;
  }
  // calculate the derivative
  derivative = error - past_error;
  // calculate the output
  out = (error * kp) + (integral * ki) + (derivative * kd);
  // make the output equal to the max value if its bigger, min value if its smaller, and itself if its inbetween
  out = out > max ? max : out < min ? min : out;
  // return the output
  return out;
}


// returns the result of the PID calculations
double PID::output() {
  double out;
  // check if the error is within the intergal limit and if it is, calculate the integral
  if (fabs(error) < integral_limit) {
    integral += error;
  }
  else {
    integral = 0;
  }
  // calculate the derivative
  derivative = error - past_error;
  // calculate the output
  out = (error * kp) + (integral * ki) + (derivative * kd);
  // make the output equal to the max value if its bigger, min value if its smaller, and itself if its inbetween
  out = out > max ? max : out < min ? min : out;
  // return the output
  return out;
}




PIDConstants::PIDConstants(double p, double i, double d, double int_limit) {
  kp = p;
  ki = i;
  kd = d;
  this->int_limit = int_limit;
}