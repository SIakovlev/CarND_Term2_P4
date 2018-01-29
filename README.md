[image1]: ./pics/cte.png

# PID control

### Introduction

Udacity [car simulator](https://github.com/udacity/self-driving-car-sim/releases)

### Building instructions

* Clone this repository
* Make a build directory: `mkdir build && cd build`
* Compile the project with `cmake .. && make`
* Run it: `./mpc`

### Implementation

There is a class called `PID`. It has two main methods: `PID::UpdateError` and `PID::Action`.

The error update is performed in`void PID::UpdateError(double cte, double dt)`. Additionaly, to deal with the [integral windup](https://en.wikipedia.org/wiki/Integral_windup), I have implemented the simplest clamping, i.e. disregarding integral terms in case the output is saturated: 

```cpp
error[2] = (cte - error[0])/dt; // derivative error
error[0] = cte;                 // proportional error
error[1] += K[1]*cte*dt;    // integral error

// Clamping
double action = -(K[0] * error[0] + error[1] + K[2]*error[2]);
if (std::fabs(action) > 1) {
  error[1] -= K[1]*cte*dt;
}
```
The other method simply calculated the resulting action based on three coefficients: `Kp, Ki, Kd`.

### Parameters tuning



### Results
