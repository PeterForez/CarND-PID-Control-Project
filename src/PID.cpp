#include "PID.h"
#include <iostream>
#include <limits>

#define TWIDDLE_STATE_INIT     (0x00)
#define TWIDDLE_STATE_STEP_ONE (0x01)
#define TWIDDLE_STATE_STEP_TWO (0x02)
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) 
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;                   // Proportional Error
  Ki = Ki_;                   // Intergral Error
  Kd = Kd_;                   // Differential Error
                              
  p_error = 0.0;              // Proportional Error
  i_error = 0.0;              // Intergral Error
  d_error = 0.0;              // Differential Error
}

void PID::UpdateError(double cte) 
{
  /**
   * TODO: Update PID errors based on cte.
   */
  
  d_error  = cte - p_error;   // Differential Error
  p_error  = cte;             // Proportional Error
  i_error += cte;             // Intergral Error
  Twiddle(cte);
}

double PID::TotalError() 
{
  /**
   * TODO: Calculate and return the total error
   */
  double steer_value = -Kp * p_error - Kd * d_error - Ki * i_error;
    
  //std::cout << " Steering Value from TotalError: " << steer_value << std::endl;
  
  std::cout << std::endl;
  std::cout << "Kp:  " << Kp << std::endl;
  std::cout << "Ki:  " << Ki << std::endl;
  std::cout << "Kd:  " << Kd << std::endl;
  std::cout << std::endl;
  
  return steer_value;  // TODO: Add your total error calc here!
}

void PID::Twiddle(double cte)
{
  double p[]  = {Kp, Ki, Kd};
  static double dp[] = {0.1*Kp, 0.1*Ki, 0.1*Kd};                 // Delta is 10 percent from the initial value
  static int i = 0;                                              // Will be used to rotate over the 3 paramters
  static int twiddle_state = TWIDDLE_STATE_INIT;                 // 
  static double best_err = std::numeric_limits<double>::max();   // best_error intialization
  
  double err = cte * cte;
  
  //std::cout << std::endl;
  //std::cout << "best_err:  " << best_err <<std::endl;
  //std::cout << std::endl;
  //
  //std::cout << std::endl;
  //std::cout << "state:  " << twiddle_state << std::endl;
  //std::cout << "dp:     " << dp[0]         << std::endl;
  //std::cout << "di:     " << dp[1]         << std::endl;
  //std::cout << "dd:     " << dp[2]         << std::endl;
  //std::cout << std::endl;
  
  if(twiddle_state == TWIDDLE_STATE_INIT)
  {
    p[i] += dp[i];
    twiddle_state = TWIDDLE_STATE_STEP_ONE;    
  }
  else if(twiddle_state == TWIDDLE_STATE_STEP_ONE)
  {
    if(err < best_err)
    {
      best_err = err;
      dp[i] *= 1.1;
      i = (i + 1) % 3;      //Go to the next parameter
      twiddle_state = TWIDDLE_STATE_INIT;    
    }
    else
    {
      p[i] -= 2 * dp[i];
      twiddle_state = TWIDDLE_STATE_STEP_TWO;
    }
  }
  else if(twiddle_state == TWIDDLE_STATE_STEP_TWO)
  {
    if(err < best_err)
    {
      best_err = err;
      dp[i] *= 1.1;
      i = (i + 1) % 3; //Go to the next parameter
      twiddle_state = TWIDDLE_STATE_INIT;
    }
    else
    {
      p[i] += dp[i];
      dp[i] *= 0.9;
      i = (i + 1) % 3; //Go to the next parameter
      twiddle_state = TWIDDLE_STATE_INIT;
    }
  }
  // Update the PID coefficients in the class
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
}