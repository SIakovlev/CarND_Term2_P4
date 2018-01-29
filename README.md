[image1]: ./pics/cte.png

# PID control

### Introduction

In this project I have implemented a PID controller in C++ to maneuver the vehicle around the track using Udacity [car simulator](https://github.com/udacity/self-driving-car-sim/releases).

### Building instructions

* Clone this repository
* Make a build directory: `mkdir build && cd build`
* Compile the project with `cmake .. && make`
* Run it: `./pid`

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
`dt` is elapsed time that is measured using c++ `chrono` library. The code snipped that does measurements is shown below:

```cpp
static double counter = 0;

finish = std::chrono::high_resolution_clock::now();
if (counter) {
  std::chrono::duration<double> dt = finish - start;
  pid.UpdateError(cte, dt.count());
}
...
...
...
start = std::chrono::high_resolution_clock::now();
ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
```

The other method simply calculated the resulting action based on three coefficients: `Kp, Ki, Kd`.

### PID parameters

#### P - proportional gain

Proportional gain is the part of controller output that is proportional to the error between current measurement and reference signal (in our case the error is `cte`). The proportional gain is not capable of controlling the car because of its inertia and external disturbances. Inertia makes it oscillating around reference point. 

#### D - derivative gain

To deal with highly inertial system we use derivative gain, that is leading in phase with respect to proportional gain (by 90 degrees). This helps to correct it when a system starts oscilating because of inertia. But there is another problem - bias, that proportional and derivative gains do not address. The contribution of P gets smaller as we reach reference, whereas D is about zero when the error remains constant.

#### I - integral gain

Integral gain helps to prevent bias problem by integrating the error over time. However, by doing so, it introduces more "inertia" in the system. Since the car has a very high inertia already, the integral term should be small enough.

#### Tuning

The tuning procedure was done manually. I set up P gain first, observed oscillations and then added D, that improved a situation, but oscillations became high frequent with less amplitude. The addition of I gain helped to remove oscillations almost completely. The final choice is: `Kp = 0.05, Ki = 0.02; Kd = 0.1`. 

### Results

The following image shows `cte` error after one encirclement of the track. Note, that at values 3-4 the car is still on the track, however it touches the red line near the ledge.

![alt text][image1]
