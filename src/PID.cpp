#include "PID.h"

#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() const {
  // P I D
  double error = (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);
  if (error > 1.0) {
    error = 1.0;
  } else if (error < (-1.0)) {
    error = -1.0;
  }

  return error;
}
