#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double K[3]) {
  Kp = K[0];
  Ki = K[1];
  Kd = K[2];

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  i_error += cte;
  d_error = cte - p_error;
  p_error = cte;
}

double PID::TotalError() {
  return (-Kp * p_error - Kd * d_error - Ki * i_error);
}
