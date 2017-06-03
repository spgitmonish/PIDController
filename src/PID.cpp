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

  // Set the steps counter to 0
  steps_counter = 0;

  // Set the sum of cte to 0.0
  sum_cte = 0.0;
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

  // Update the sum of cte
  sum_cte += cte;

  // Update the propotional component error(Set to the current cross track error)
  p_error = cte;

  // Update the integation component error(Sum of the cross track errors)
  i_error = sum_cte;

  // Update the differentiation component error(Difference of current & previous)
  d_error = cte - prev_cte;

  // Store the previous cte for next iteration
  prev_cte = cte;

#if SGD
  // Push back the expected steering angle
  sgd_y.push_back(cte);

  // Create vector for 'x' values in h(x)
  vector<double> x_values{p_error, i_error, d_error};

  // Push the vector into the vector of vectors
  sgd_h_x.push_back(x_values);
#endif
}

// Calculates the total PID error
void PID::TotalError(double cte)
{
#if TWIDDLE
  if(steps_counter == 50)
  {
    // Update the individual error components
    UpdateError(cte);

    // Use the twiddle algorithm to calculate the best coefficients
    Twiddle(cte);

    // Reinitialze the counter
    steps_counter = 0;

  #if DEBUG
    cout << "Kp: " << Kp << ", Ki: " << Ki << ",Kd: " << Kd << endl;
  #endif
  }
#elif SGD
  // Call update error function with the latest cte
  UpdateError(cte);

  // If the step count is equal to 100 or if cross track error switches on us
  if(steps_counter == 100)
  {
    // Call the SGD algorithm to calculate the coefficients
    StochasticGradientDescent();

    // Reset the counter to 0
    steps_counter = 0;

    // Reset the variables involved and the vectors
    prev_cte = DBL_MAX;
    sum_cte = 0.0;
    sgd_y.clear();
    sgd_h_x.clear();

  #if DEBUG
    cout << "Kp: " << Kp << ", Ki: " << Ki << ",Kd: " << Kd << endl;
  #endif
  }
#endif

  // Increment the counter
  steps_counter += 1;
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

#if DEBUG_VERBOSE
  cout << "Before(Kp: " << coefficients[0] << ", Ki: " << coefficients[1] << ", Kd: " << coefficients[2] << ")" <<endl;
#endif

  // Initialize the vector of potential changes to the coefficients
  vector<double> potential_coefficients{0.1, 0.1, 0.1};

  // Sum of the potential coefficient changes
  // NOTE: The accumulate function from the std algorithm library is used
  double sum_of_pot_coeffs = accumulate(potential_coefficients.begin(),
                                        potential_coefficients.end(),
                                        0.0);

#if DEBUG_VERBOSE
  cout << "SoPC: " << sum_of_pot_coeffs << endl;
#endif

  // Variable to store the best error calculated
  double best_error = -(coefficients[0] * p_error) \
                      -(coefficients[1] * i_error) \
                      -(coefficients[2] * d_error);

#if DEBUG_VERBOSE
  cout << "BE: " << best_error << endl;
#endif

  // Twiddle algorithm goes till the sum of the potential is less than a threshold
  //while(sum_of_pot_coeffs > 0.2)
  sum_of_pot_coeffs = 0.0;

  while(sum_of_pot_coeffs < 15)
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
      #if DEBUG_VERBOSE
       cout << "Coeff[" << coeff_index << "]: " << coefficients[coeff_index] << endl;
       cout << "NE: " << new_error << endl;
      #endif

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
    /*sum_of_pot_coeffs = accumulate(potential_coefficients.begin(),
                                   potential_coefficients.end(),
                                   0.0);*/
    sum_of_pot_coeffs += 1;
  }

  // Update the respective coefficients values after the twiddle calculation
  Kp = coefficients[0];
  Ki = coefficients[1];
  Kd = coefficients[2];

#if DEBUG_VERBOSE
  cout << "After(Kp: " << coefficients[0] << ", Ki: " << coefficients[1] << ", Kd: " << coefficients[2] << ")" <<endl;
#endif
}

// Function which calculates the coefficients using SGD
void PID::StochasticGradientDescent(void)
{
  // Vector of inital coefficients
  vector<double> coefficients{Kp, Ki, Kd};

  // Learning rate and number of epochs
  double alpha = 0.0003;
  int epochs = 200;

#if DEBUG_VERBOSE
  cout << "Before(Kp: " << coefficients[0] << ", Ki: " << coefficients[1] << ", Kd: " << coefficients[2] << ")" <<endl;
#endif

  for(size_t epoch = 0; epoch < epochs; epoch++)
  {
    // Sum of squared errors
    double sum_of_sq_error = 0.0;

    // Calculate the error for each of the cases in the "training set"
    for(size_t index = 0; index < sgd_y.size(); index++)
    {
      // Predicted value
      double predicted_steer = coefficients[0] * sgd_h_x[index][0] + \
                               coefficients[1] * sgd_h_x[index][1] + \
                               coefficients[2] * sgd_h_x[index][2];

    #if DEBUG_VERBOSE
      cout << "PS: " << predicted_steer << endl;
    #endif

      // Error between the expected value and actual value
      double error = predicted_steer - sgd_y[index];

    #if DEBUG_VERBOSE
      cout << "Err: " << error << endl;
    #endif

      // Add up the sum of the squared errors
      sum_of_sq_error += error * error;

      // Update each of the coefficients for this sample in the epoch
      for(size_t coeff_index = 0; coeff_index < coefficients.size(); coeff_index++)
      {
        coefficients[coeff_index] = coefficients[coeff_index] - alpha * error * sgd_h_x[index][coeff_index];
      }
    }
  }

#if DEBUG_VERBOSE
  cout << "After(Kp: " << coefficients[0] << ", Ki: " << coefficients[1] << ", Kd: " << coefficients[2] << ")" <<endl;
#endif

  // Update the coefficients of the PID object after SGD
  Kp = coefficients[0];
  Ki = coefficients[1];
  Kd = coefficients[2];
}
