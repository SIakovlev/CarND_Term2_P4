#ifndef PID_H
#define PID_H

#include "Eigen/Dense"
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  std::vector<double> error = {0.0, 0.0, 0.0};

  /*
  * Coefficients
  */
  std::vector<double> K = {0.0, 0.0, 0.0};

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte, double dt);

  /*
  * Calculate the total PID error.
  */
  //double TotalError();

  double Action();

  void Twiddle(double);
};

#endif /* PID_H */
