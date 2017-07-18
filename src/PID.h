#ifndef PID_H
#define PID_H

#include <string>
#include <fstream>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double K[3];

  // Twiddle 
  double dp[3];
  double ss_error;
  double best_error;
  int steps;
  int index;
  int check;
  double best_coeff[3];
  int untracked_steps;
  int min_tracked_steps;
  double correct_dp_increase;
  double incorrect_dp_decrease;
  double max_cte;
  double normalized_ss_error;
  double best_params[3];

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double p[]);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Adjust parameters using twiddle. Returns sum of dp array.
  */
  void Twiddle();

  /*
  * Saves coefficients of best fit
  */
  void SaveCoefficients();

  bool CheckBetter();

  void ResetErrors();
  void EarlyReset(int twiddle_steps);
};

#endif /* PID_H */
