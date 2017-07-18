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

  /*
  * TWIDDLE
  */
  // amount to adjust coefficients by
  double dp[3];
  // saves best coefficients to run after twiddle completes
  double best_coeffs[3];
  // sum of squared errors
  double ss_error;
  // normalized by the number of steps, can compare two parameter sets that do
  // not complete
  double normalized_ss_error;
  // stores best normalized_ss_error
  double best_error;
  // tracks furthest distance the car strays from center, another way to view
  // error
  double max_cte;
  // tracks the number of steps taken
  int steps;
  // next parameter to tune
  int index;
  // controls add/sub adjustment of parameter
  int check;
  // allows for initial steps not to be counted in ss_error
  int untracked_steps;
  // minimum number of steps beyond initial untracked steps for error to count
  // reconciled some early reset issues, likely no longer needed
  int min_tracked_steps;
  // for tuning of twiddle, should be > 1
  double correct_dp_increase;
  // for tuning of twiddle, should be < 1
  double incorrect_dp_decrease;




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
  * Saves coefficients of best fit and prints to terminal
  */
  void SaveCoefficients();

  /*
  * First check in twiddle to see if error was better
  */
  bool CheckBetter();

  /*
  * Prints previous errors and resets all error variables except best error in
  * anticipation of new parameters to test
  */
  void ResetErrors();

  /*
  * Resets sim and twiddle variables when car gets too far from center. 
  * adds a large number to ss_error to ensure its not picked over a 
  * parameter set that completes twiddle_steps
  */
  void EarlyReset(int twiddle_steps);
};

#endif /* PID_H */
