#include <iostream>
#include <math.h>
#include <float.h>
#include <numeric>
#include <vector>

#ifndef PID_H
#define PID_H

#define DEBUG 1
#define DEBUG_VERBOSE (DEBUG && 0)
#define TWIDDLE 1
#define SGD 0

// Make sure a compile time assert is thrown to prevent both algorithms from
// being turned ON at the same time
#if TWIDDLE && SGD
  #error "Only one of the algorithm flags can be enabled"
#endif

using namespace std;

// Enum to indicate whether the coefficients were moved up or down and need
// to be validated accordingly
enum coefficient_update {START, UP, DOWN, MUL_1_1, MUL_0_9};

// Type of PID
enum pid_type {STEERING, THROTTLE};

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
  void Init(double Kp_to_set, double Ki_to_set, double Kd_to_set, pid_type type_of_pid_to_set);

  // Updates the PID error variables given cross track error
  void UpdateError(double cte);

  // Calculates the total PID error
  void TotalError(double cte);
private:
  // Private variable which keeps track of the previous cross track error
  double prev_cte;

  // Counter which keeps track of the number of steps
  int steps_counter;

  // Variable to keep track of the sum of cross track errors
  double sum_cte;

  // Type of PID(Throttle or Steering)
  pid_type type_of_pid;
#if SGD
  // Vector which keeps track of the steering angle expected 'y'
  vector<double> sgd_y;

  // Vector of vectors of the errors calculated
  vector<vector<double>> sgd_h_x;

  // Constant for determining number of steps before SGD kicks back in
  const int steps_threshold = 100;

  // Function which calculates the coefficients using SGD
  void StochasticGradientDescent(void);
#endif
#if TWIDDLE
  // Private variable to keep track of the total error accumulated
  double total_error;

  // Variable to store the best error(if it changes) for twiddle call
  double best_error;

  // Enum variable for setting the correct update to do
  coefficient_update coeff_update;

  // Variable which tracks potential changes to apply to the coefficients
  vector<double> potential_coefficients;

  // Current coefficient change to be tested
  int current_coefficient;

  // Constant for determining number of steps before Twiddle kicks back in
  const int steps_threshold = 300;

  // Function which uses the twiddle algorithm to calculate the coefficients
  // for the respective components using the passed in cross track error
  void Twiddle(double cte);
#endif
};

#endif /* PID_H */
