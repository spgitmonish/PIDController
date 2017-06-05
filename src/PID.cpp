#include "PID.h"
using namespace std;

PID::PID() {}

PID::~PID() {}

// Initializes the PID controller
void PID::Init(double Kp_to_set, double Ki_to_set, double Kd_to_set, pid_type type_of_pid_to_set)
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

  // Set the type of PID initiated
  type_of_pid = type_of_pid_to_set;

#if TWIDDLE
  // Set the initial error to 0.0
  total_error = 0.0;

  // Set best error to a very high value
  best_error = DBL_MAX;

  // Set up the potential coefficient values to change
  potential_coefficients.push_back(0.125);
  potential_coefficients.push_back(0.0001);
  potential_coefficients.push_back(0.30);

  // Always start with the first coefficient
  current_coefficient = 0;

  // Set the enum to indicate start
  coeff_update = START;
#endif
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

// Variables related to Stochastic Gradient Descent
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
  // Call update error function with the latest cte
  UpdateError(cte);

#if SGD
  // If the step count is equal to 100 or if cross track error switches on us
  if(steps_counter == steps_threshold)
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
#elif TWIDDLE
  if(steps_counter >= steps_threshold / 2)
  {
    // Accumulate error at the rate of power of two of cte because error hasn't
    // been accounted for in the iterations, to allow time for converging
    total_error += pow(cte, 2);
  }
  if(steps_counter == steps_threshold)
  {
    // Use the twiddle algorithm to calculate the best coefficients
    Twiddle(cte);

    // Reset the variables involved
    prev_cte = DBL_MAX;
    sum_cte = 0.0;
    total_error = 0.0;

    // Reinitialze the counter
    steps_counter = 0;

  #if DEBUG
    if(type_of_pid == STEERING)
    {
      cout << "Kp(S): " << Kp << ", Ki(S): " << Ki << ", Kd(S): " << Kd << endl;
    }
    else if(type_of_pid == THROTTLE)
    {
      cout << "Kp(T): " << Kp << ", Ki(T): " << Ki << ", Kd(T): " << Kd << endl;
    }
  #endif
  }
#endif

  // Increment the counter
  steps_counter += 1;
}

#if SGD
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

      // If the error has converged, break, to save computation time
      if(sum_of_sq_error < 0.00001)
      {
        break;
      }

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
#endif

#if TWIDDLE
// Function which uses the twiddle algorithm to calculate the coefficients
// for the respective components using the passed in cross track error
void PID::Twiddle(double cte)
{
  // Calculate the sum of potential changes, this variable decides if we
  // need to stop twiddling
  double sum_of_pot_coefficients = accumulate(potential_coefficients.begin(),
                                              potential_coefficients.end(),
                                              0.0);

  // Check if the sum of potential changes is very close to 0, that means, starting
  // from 1, we went all the way down to almost zero
  if(sum_of_pot_coefficients > 0.000001)
  {
    // Get the latest error
    double new_error = total_error;

    if(coeff_update == UP) // The coefficient value was incremented
    {
      // Check if the previous update to the coefficient improved the error
      if(new_error < best_error)
      {
        // New best error
        best_error = new_error;

        // Indicate the multiplier update was applied to this potential coefficient
        potential_coefficients[current_coefficient] *= 1.1;

        // Set the enum to indicate higher factor multiplication was applied
        coeff_update = MUL_1_1;

        // Move onto the next coefficient as the increment to the previous coefficient
        // did lead to an improvement in the error
        current_coefficient = (current_coefficient + 1) % 3;

        // Return from this function
        return;
      }
      else
      {
        // The increment for this coefficient didn't lead to an improvement
        if(current_coefficient == 0)
        {
          // Decrement Kp
          Kp -= 2 * potential_coefficients[current_coefficient];
        }
        else if(current_coefficient == 1)
        {
          // Decrement Ki
          Ki -= 2 * potential_coefficients[current_coefficient];
        }
        else
        {
          // Decrement Kd
          Kd -= 2 * potential_coefficients[current_coefficient];
        }

        // Mark a flag which indicates that the decrement needs to be validated
        coeff_update = DOWN;

        // Return at this point
        return;
      }
    }
    else if(coeff_update == DOWN) // The coefficient value was reduced
    {
      // Check if the decrement to the coefficient improved the error
      if(new_error < best_error)
      {
        // New best error
        best_error = new_error;

        // Indicate the multiplier update was applied to this potential coefficient
        potential_coefficients[current_coefficient] *= 1.1;

        // Set the enum to indicate higher factor multiplication was applied
        coeff_update = MUL_1_1;

        // Move onto the next coefficient as the update to the previous coefficient
        // did lead to an improvement in the error
        current_coefficient = (current_coefficient + 1) % 3;

        // Return from this function
        return;
      }
      else
      {
        // Go back the original values by incrementing
        if(current_coefficient == 0)
        {
          // Incemement Kp
          Kp += potential_coefficients[current_coefficient];
        }
        else if(current_coefficient == 1)
        {
          // Incemenent Ki
          Ki += potential_coefficients[current_coefficient];
        }
        else
        {
          // Increment Kd
          Kd += potential_coefficients[current_coefficient];
        }

        // Multiply big a smaller factor
        potential_coefficients[current_coefficient] *= 0.9;

        // Set the enum to indicate smaller factor multiplication was applied
        coeff_update = MUL_0_9;

        // Move onto the next coefficient as the update to the previous coefficient
        // did lead to an improvement in the error
        current_coefficient = (current_coefficient + 1) % 3;

        // Return from this function
        return;
      }
    }
    else // The update process must have started or a factor multiplication was applied
    {
      if(current_coefficient == 0)
      {
        // Increment Kp
        Kp += potential_coefficients[current_coefficient];
      }
      else if(current_coefficient == 1)
      {
        // Increment Ki
        Ki += potential_coefficients[current_coefficient];
      }
      else
      {
        // Increment Kd
        Kd += potential_coefficients[current_coefficient];
      }

      // Indicate that the coefficients were incremented
      coeff_update = UP;
    }
  }
  else
  {
    cout << "Not twiddling" << endl;
  }
}
#endif
