#include "PID.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace std;

enum States { start_alg, start_check, complete_check };

inline double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

/*
* TODO: Complete the PID class.
*/

PID::PID() {
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  K = {Kp, Ki, Kd};
}

void PID::UpdateError(double cte, double dt) {

  error[2] = (cte - error[0])/dt; // derivative error
  error[0] = cte;                 // proportional error
  error[1] += K[1]*cte*dt;    // integral error
  
  
  double action = -(K[0] * error[0] + error[1] + K[2]*error[2]);
  if (std::fabs(action) > 1) {
    error[1] -= K[1]*cte*dt;
  }

  std::cout << "P error contribution: " << error[0] << " ";
  std::cout << "I error contribution: " << error[1] << " ";
  std::cout << "D error contribution: " << error[2] << std::endl;
}

double PID::Action() {
  double action = -(K[0] * error[0] + error[1] + K[2]*error[2]);
  return action;
}

// For future experiments. It works, but completely useless here
void PID::Twiddle(double err) {

  std::cout << "Accumulative error: " << err << std::endl;

  static std::vector<double> delta = {0.01, 0.0001, 0.01};
  static unsigned long i = 0;
  static double best_err = err;
  static States s = start_alg;

  std::cout << "dKp=" << delta[0] << "; dKi=" << delta[1] << "; dKd=" << delta[2] << std::endl;
  switch(s) {

    case start_alg: {
      K[i] += delta[i];
      s = start_check;
      return; // Run
    }

    case start_check: {
      if (err < best_err) {
        best_err = err;
        delta[i] *= 1.2;
        i = (i+1)%K.size(); // to prevent i > 3
        s = start_alg;
      } else {
        K[i] -= 2*delta[i];
        s = complete_check;
      }
      return; //Run
    }

    case complete_check: {
      if (err < best_err) {
        best_err = err;
        delta[i] *= 1.2;
      } else {
        K[i] += delta[i];
        delta[i] *= 0.8;
      }
      i = (i+1)%K.size();
      s = start_alg;
      return;
    }

    default: {
      if (std::accumulate(delta.begin(), delta.end(), 0.0) < 0.0001 ) return;
      else std::cout << "!?" << std::endl;
    }
  }
}


