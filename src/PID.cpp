#include "PID.h"
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

// Initializes the PID controller
void PID::Init(double Kp_to_set, double Ki_to_set, double Kd_to_set)
{
  // Set the respective components constants
  Kp = Kp_to_set;
  Ki = Ki_to_set;
  Kd = Kd_to_set;

  // Set the initial error components to 0.0
  p_error = i_error = d_error = 0.0;

  // Set the previous cross track error to the maximum possible value
  // This is just during initalization
  prev_cte = DBL_MAX;
}

// Updates the PID error variables given cross track error
void PID::UpdateError(double cte)
{
  // If this is the first error calculation, then the previous cross track error
  // is the same as the incoming cross track error
  if(prev_cte == DBL_MAX)
  {
    prev_cte = cte;
  }

  // Update the propotional component error(Set to the current cross track error)
  p_error = cte;

  // Update the integation component error(Sum of the cross track errors)
  i_error += cte;

  // Update the differentiation component error(Difference of current & previous)
  d_error = cte - prev_cte;
}

// Calculates the total PID error
double PID::TotalError(double cte)
{
  // Variable to store the total error after calculating the coefficients
  double total_error;

  // Use the twiddle algorithm to calculate the best coefficients
  Twiddle(cte);

  // Update the individual error components
  UpdateError(cte);

  // Calculate the total error with the calculated coefficients
  total_error = -(Kp * p_error) - (Ki * i_error) - (Kd * d_error);

  // Return the new error calculated
  return total_error;
}

// Function which uses the twiddle algorithm to calculate the coefficients
// for the respective components using the passed in cross track error
void PID::Twiddle(double cte)
{
  // Vector of coefficients of the respective components
  // NOTE: For the first calculation all the components are set 0.0 because
  //       when the Init() function is called from main for the 1st time all
  //       coefficients are set to 0.0
  vector<double> coefficients{Kp, Ki, Kd};

  // Initialize the vector of potential changes to the coefficients
  vector<double> potential_coefficients{1.0, 1.0, 1.0};

  // Sum of the potential coefficient changes
  // NOTE: The accumulate function from the std algorithm library is used
  double sum_of_pot_coeffs = accumulate(potential_coefficients.begin(),
                                        potential_coefficients.end(),
                                        0.0);

  // Variable to store the best error calculated
  double best_error = -(Kp * p_error) - (Ki * i_error) - (Kd * d_error);

  // Twiddle algorithm goes till the sum of the potential is less than a threshold
  while(sum_of_pot_coeffs > 0.2)
  {
    // Go through one component's coefficient at a time
    for(size_t coeff_index = 0; coeff_index < coefficients.size(); coeff_index++)
    {
      // Increment the coefficient's value by the respective
      // potential_coefficients value
      coefficients[coeff_index] += potential_coefficients[coeff_index];

      // Calculate the error with this change in the coefficient's value
      double new_error = -(coefficients[0] * p_error) \
                         -(coefficients[1] * i_error) \
                         -(coefficients[2] * d_error);

      // Check if the change in the coefficient's value is better than
      // previous best error
      if(new_error < best_error)
      {
        // Update the best error variable
        best_error = new_error;

        // Change the potential_coefficients's value by multiplying 1.1
        potential_coefficients[coeff_index] *= 1.1;
      }
      else
      {
        // Reduce the coefficient's value by twice
        // the potential_coefficients's value as the coefficient's value was
        // incremented at the beginning of the loop
        coefficients[coeff_index] -= 2.0 * potential_coefficients[coeff_index];\

        // Calculate the error with this change in the coefficient's value
        new_error = -(coefficients[0] * p_error) \
                    -(coefficients[1] * i_error) \
                    -(coefficients[2] * d_error);

        // Check if the decrease in the coefficient's value leads to better error
        if(new_error < best_error)
        {
          // Update the best error variable
          best_error = new_error;

          // Change the potential_coefficients's value by multiplying 1.1
          potential_coefficients[coeff_index] *= 1.1;
        }
        else
        {
          // Go back to the original value of the coefficient
          coefficients[coeff_index] += potential_coefficients[coeff_index];

          // Change the potential_coefficients's value by multiplying 0.9
          potential_coefficients[coeff_index] *= 0.9;
        }
      }
    }

    // Recalculate the sum of potential_coefficients values
    sum_of_pot_coeffs = accumulate(potential_coefficients.begin(),
                                   potential_coefficients.end(),
                                   0.0);

  }

  // Update the respective coefficients values after the twiddle calculation
  Kp = coefficients[0];
  Ki = coefficients[1];
  Kd = coefficients[2];
}
