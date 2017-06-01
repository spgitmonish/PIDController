#include <iostream>
#include <math.h>
#include <float.h>
#include <numeric>
#include <vector>

#ifndef PID_H
#define PID_H

#define DEBUG 0

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

  // Function which uses the twiddle algorithm to calculate the coefficients
  // for the respective components using the passed in cross track error
  void Twiddle(double cte);
};

#endif /* PID_H */
