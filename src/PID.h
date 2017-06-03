#include <iostream>
#include <math.h>
#include <float.h>
#include <numeric>
#include <vector>

#ifndef PID_H
#define PID_H

#define DEBUG 1
#define DEBUG_VERBOSE (DEBUG && 0)
#define TWIDDLE 0
#define SGD 1

using namespace std;

class PID
{
public:
  // Error Variables for Propotional, Integral and Differentiate components
  double p_error;
  double i_error;
  double d_error;

  // Coefficient Variables for Propotional, Integral and Differentiate components
  double Kp;
  double Ki;
  double Kd;

  // Constructor
  PID();

  // Destructor
  virtual ~PID();

  // Initializes the PID controller
  void Init(double Kp_to_set, double Ki_to_set, double Kd_to_set);

  // Updates the PID error variables given cross track error
  void UpdateError(double cte);

  // Calculates the total PID error
  double TotalError(double cte);
private:
  // Private variable which keeps track of the previous cross track error
  double prev_cte;

  // Counter which keeps track of the number of steps
  int steps_counter;

  // Variable to keep track of the sum of cross track errors
  double sum_cte;

#if SGD
  // Vector which keeps track of the steering angle expected 'y'
  vector<double> sgd_y;

  // Vector of vectors of the errors calculated
  vector<vector<double>> sgd_h_x;
#endif

  // Function which uses the twiddle algorithm to calculate the coefficients
  // for the respective components using the passed in cross track error
  void Twiddle(double cte);

  // Function which calculates the coefficients using SGD
  void StochasticGradientDescent(void);
};

#endif /* PID_H */
