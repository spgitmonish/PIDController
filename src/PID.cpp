#include "PID.h"
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

// Initializes the PID controller
void PID::Init(double KpToSet, double KiToSet, double KdToSet)
{
  // Set the respective components constants
  Kp = KpToSet;
  Ki = KiToSet;
  Kd = KdToSet;

  // Set the initial error components to 0.0
  p_error = i_error = d_error = 0.0;

  // Set the previous cross track error to the maximum possible value
  // This is just during initalization
  prev_cte = DBL_MAX;
}

// Updates the PID error variables given cross track error
void PID::UpdateError(double cte)
{
}

// Calculates the total PID error
double PID::TotalError()
{
  return 0.0;
}
