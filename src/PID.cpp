#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp = Kp;
  Ki = Ki;
  Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  double previous_cte = p_error;

  // Proportional error is just the CTE (Cross Track Error)
  p_error = cte;

  // Integration error is the sum of all CTE so far
  i_error = i_error + cte;

  // Differential error is the rate of change of the CTE but assuming
  // each call is 1 time-step then it simplifies to subtracting the 
  // previous CTE from the current CTE.
  d_error = cte - previous_cte;
}

double PID::TotalError() {
  // Multiple out our errros by the coefficients
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}

