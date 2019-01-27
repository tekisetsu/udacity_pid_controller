##Omar Benzakour
###PID controller
---

**Abstract**

The goals / steps of this project are the following:

* Implement a PID controller that will allow a car to drive autonomously when provided with a cross track error

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from [here](https://github.com/udacity/self-driving-car-sim/releases)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Description of the PID controller


A proportional–integral–derivative controller (PID controller or three term controller) is a control loop feedback mechanism widely used in industrial control systems and a variety of other applications requiring continuously modulated control. A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively) which give the controller its name. ([wikipedia](https://en.wikipedia.org/wiki/PID_controller))

### Proportional parameter

The first component is the proportional parameter. This parameter aims to reduce the error by applying a correction proporional to the current error and can be reprensented by the following formula

	correction = -Kp * error 
In our scenario, this component is going to steer the wheel in the direction of the track we aim to meet. Because of its adjusting nature, with this component alone we are not able to stay on track. The resulting path is a sinusoide.


### Derivative parameter

The second component is the derivative parameter and applies a correction that is proportional to the rate of change to 0. Therefore this parameter doesn't aim to correct the error but aims to avoid the overshoots of the proportional parameter

	correction = -Kd * d(error)/dt 
In our scenario, this component is going to reduce the steering created by the propotional parameter. The resulting path will therefore look like an exponential.

### Integral parameter

The last component is the integral parameter and aims to correct errors that are persisting. Mathematicaly, the correction provided is proportional to the sum of the errors which corresponds to the integrale in discrete systems

	correction = -Ki * ∑ error 
In our scenario, this component is going to reduce the steering created by the propotional parameter. The resulting path will therefore look like an exponential.


## Finding appropriate hyperparameters

Now that we have coded our PID controller we could have implemented the gradient descent algorithm to get the optimal hyperparameters. This algorithm is time consuming because the simulation lasts more than 30s. Instead I will take a more empirical approach

The term that corrects direcly the error is the proportional term. It is therefore important for Kp to be big enough so that it could steer the wheel. I will therefore initialize it to 1 and see if Kp manages to steer the weels. 

You can see the result in the kp video. At first, in a straight road, the car is stable. Then when we road is turning, the car overshoots and becomes unstable. We therefore know that Kp = 1 allows the proportional parameter to strongly steer the wheel and we will now correct this term with Kd.

I have started with Kd = 1, the overshoot was still strong. Then I have tried with Kd=10 which was better but still not perfect. Then I have tried with Kd=100 and obtained a good result. The car was was able to finish the circuit without going off track.

In the turn we can notice that the steering is a bit too strong. I have therefore reduced Kp from 1 to 0.75 and it smoothed the turns

I didn't find the need to have an integral parameter. Maybe there is no persisting error.


## improvements

The implemented PID controller allows the car to finish the circuit without going of track. Nonetheless the is still a room for improvements and better tunning, indeed the the turns are not that smooth. We could have for example used the gradient descent algorithm to find better hyperparameters
