#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  dp[0] = 0.18;
  dp[1] = 0.01;
  dp[2] = 2.11;

  untracked_steps = 100;
  min_tracked_steps = 100;
  correct_dp_increase = 1.1;
  incorrect_dp_decrease = 0.9;

  check = 0;
  index = 0;
  steps = 0;

  best_error = 1e8;
  normalized_ss_error = 0.0;
  max_cte = 0.0;
}

PID::~PID() {}

void PID::Init(double p[]) {
  for (int i = 0; i < 3; i++) {
    K[i] = p[i]; 
  }

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  ss_error = 0.0;
}

void PID::UpdateError(double cte) {
  i_error += cte;
  d_error = cte - p_error;
  p_error = cte;
  if (steps > untracked_steps) {ss_error += cte*cte;}
  if (fabs(cte) > max_cte && steps > untracked_steps) {max_cte = fabs(cte);}
  steps++;
}

void PID::ResetErrors() {
  cout<<"Normalized error: "<<normalized_ss_error<<"  Best:"<<best_error<<endl;
  cout<<"Max CTE: "<< max_cte<<std::endl;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  ss_error = 0.0;
  normalized_ss_error = 0.0;
  max_cte = 0.0;
}

double PID::TotalError() {
  return (-K[0] * p_error - K[2] * d_error - K[1] * i_error);
}

bool PID::CheckBetter() {
  if (normalized_ss_error < best_error) {
    steps = 0;
    best_error = normalized_ss_error;
    SaveCoefficients();
    dp[index] *= correct_dp_increase;
    index = (index + 1) % 3;
    K[index] += dp[index];
    check = 1;
    ResetErrors();
    return true;
  } else {
    return false;
  }
}

void PID::SaveCoefficients() {
  std::copy(std::begin(K), std::end(K), std::begin(best_coeffs));
  ofstream ofs("best_coeff.txt", ios::out| ios::app);
  ofs<<"Coefficients: (P) "<<K[0]<<" (I) "<<K[1]<<" (D) "<<K[2]<<"\r\n";
  ofs<<"DP: (P) "<<dp[0]<<" (I) "<<dp[1]<<" (D) "<<dp[2]<<"\r\n";
  ofs<<"Best error: "<<best_error<<"   Max CTE: "<<max_cte<<"\r\n\r\n";
  ofs.close();
}

void PID::Twiddle() {
  // normalizes the sum of squared errors if the minimum number of tracked
  // steps is met, otherwise sets ss_errors to a large value
  if (steps > (untracked_steps + min_tracked_steps)) {
    normalized_ss_error = ss_error / (steps - untracked_steps);
  } else {
    ss_error = 1e8;
  }

  // for the first run through
  if (check == 0) {
    steps = 0;
    best_error = normalized_ss_error;
    SaveCoefficients();
    K[index] += dp[index];
    check = 1;
    ResetErrors();
    return;
  }

  // checks parameter increase for new best, otherwise decreases parameter
  if (check == 1) {
    if (CheckBetter() == true) {
      return;
    } else {
      steps = 0;
      K[index] -= 2*dp[index];
      check = 2;
      ResetErrors();
      return;
    }
  }

  // checks parameter decrease for new best, otherwise resets parameter and
  // increase the nex parameter to be tuned
  if (check == 2) {
    if (CheckBetter() == true) {
      check = 1;
      return;
  } else {
    steps = 0;
    K[index] += dp[index];
    dp[index] *= incorrect_dp_decrease;
    index = (index + 1) % 3;
    K[index] += dp[index];
    check = 1;
    ResetErrors();
    return;
    }
  }
}

void PID::EarlyReset(int twiddle_steps) {
  std::cout<<"RESTARTED EARLY - CTE TO HIGH"<<std::endl;
  ss_error = 100 * twiddle_steps;
  Twiddle();
}