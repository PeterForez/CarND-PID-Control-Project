#include "PID.h"
#include <iostream>

#define PROPORTIONAL (0x00U)
#define INTERGRAL    (0x01U)
#define DIFFERENTIAL (0x02U)
#define TWIDDLE_TOLERENCE (0)

#define TWIDDLE_STATE_INIT (0x00)
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
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) 
{
  /**
   * TODO: Update PID errors based on cte.
   */
  
  d_error  = cte - p_error;
  p_error  = cte;
  i_error += cte;
  Twiddle();
}

double PID::TotalError() 
{
  /**
   * TODO: Calculate and return the total error
   */
  double steer_value = -Kp * p_error - Kd * d_error - Ki * i_error;
    
  std::cout << " Steering Value from TotalError: " << steer_value << std::endl;
  return steer_value;  // TODO: Add your total error calc here!
}


//Best Resources: https://knowledge.udacity.com/questions/411699
void PID::Twiddle() 
{
  double p[]  = {Kp, Ki, Kd};
  //static double dp[] = {1, 1, 1};
  //static double dp[] = {0.5, 0.5, 0.5};
  static double dp[] = {0.01, 0.0001, 0.1}; //Delta is 10 percent from the initial value
  static int index = 0; // will be used to rotate over the 3 paramters
  static int state = TWIDDLE_STATE_INIT; //
  static double best_err;
  double err;
  
  std::cout << std::endl;
  std::cout << "best_err:  " << best_err <<std::endl;
  std::cout << "Kp:  " << Kp << std::endl;
  std::cout << "Ki:  " << Ki << std::endl;
  std::cout << "Kd:  " << Kd << std::endl;
  std::cout << std::endl;
  
  std::cout << std::endl;
  std::cout << "state:  " << state << std::endl;
  std::cout << "dp:  " << dp[0] << std::endl;
  std::cout << "di:  " << dp[1] << std::endl;
  std::cout << "dd:  " << dp[2] << std::endl;
  std::cout << std::endl;
  
  if(state == TWIDDLE_STATE_INIT)
  {
    err = TotalError() * TotalError();
    best_err = err;
    p[index] += dp[index];
    state = 1;    
  }
  else if (dp[0] + dp[0] + dp[0] > TWIDDLE_TOLERENCE)
  {
    if(state == 1)
    {
      err = TotalError() * TotalError();
      if(err < best_err)
      {
        best_err = err;
        dp[index] *= 1.1;
        index = (index + 1) % 3; //Go to the next parameter
        p[index] += dp[index];      
      }
      else
      {
        p[index] -= 2 * dp[index];
        if(p[index] < 0)
        {
          p[index] = 0;
        }
        state = 2;
      }
    }
    else if(state == 2)
    {
      err = TotalError() * TotalError();
      if(err < best_err)
      {
        best_err = err;
        dp[index] *= 1.1;
        index = (index + 1) % 3; //Go to the next parameter
        p[index] += dp[index];
        state = 1;
      }
      else
      {
        p[index] += dp[index];
        dp[index] *= 0.9;
        index = (index + 1) % 3; //Go to the next parameter
        p[index] += dp[index];
        state = 1;
      }
    }
  }
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
}


//void PID::Twiddle(double cte) 
//{
//  double p = [Kp, Ki, Kd];
//  double dp = [1, 1, 1];
//  
//  static double Dp = 1;
//  static double Di = 1;
//  static double Dd = 1;
//  
//  static double err = 0;
//  static int Parameter = PROPORTIONAL;
//  
//  double best_error = cte * cte;
//  
//  if ((Dp + Di + Dd) > 0.2)
//  {
//    switch(Parameter)
//    {
//      case PROPORTIONAL:
//        Kp += Dp;
//        
//        //Parameter = INTERGRAL;
//        
//        
//        break;
//      case INTERGRAL:
//        Parameter = DIFFERENTIAL;
//        break;
//      case DIFFERENTIAL:
//        Parameter = PROPORTIONAL;
//        break;
//    }
//  }
//}






