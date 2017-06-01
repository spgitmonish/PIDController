#include <iostream>
#include <math.h>
#include <float.h>

#ifndef PID_H
#define PID_H

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
  void Init(double KpToSet, double KiToSet, double KdToSet);

  // Updates the PID error variables given cross track error
  void UpdateError(double cte);

  // Calculates the total PID error
  double TotalError();
private:
  // Private variable which keeps track of the previous cross track error
  double prev_cte;
};

#endif /* PID_H */
