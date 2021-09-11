- [CarND-Controls-PID](#carnd-controls-pid)
  - [Dependencies](#dependencies)
  - [Basic Build Instructions](#basic-build-instructions)
  - [Editor Settings](#editor-settings)
  - [Code Style](#code-style)
  - [Project Instructions and Rubric](#project-instructions-and-rubric)
  - [Hints!](#hints)
  - [Call for IDE Profiles Pull Requests](#call-for-ide-profiles-pull-requests)
  - [How to write a README](#how-to-write-a-readme)
- [Implemenation of CarND-Controls-PID](#implemenation-of-carnd-controls-pid)
  - [Overview](#overview)
    - [What is PID Controller?](#what-is-pid-controller)
  - [Tuning](#tuning)
  - [Maximum Steering angel](#maximum-steering-angel)
  - [Increase the Velocity of the Car](#increase-the-velocity-of-the-car)
- [Reference](#reference)
# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program


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
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).



# Implemenation of CarND-Controls-PID
## Overview
The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

The goal of this project is to implement a PID controller in C++ to maneuver the vehicle around the track!

**Project Steps:**
* Implement PID controller for car's sterring 
* Fine tuning the hyperparmater of the controller

### What is PID Controller?

A proportional–integral–derivative controller (PID controller or three-term controller) is a control loop mechanism employing feedback that is widely used in industrial control systems and a variety of other applications requiring continuously modulated control. 

A PID controller continuously calculates an error value ${\displaystyle e(t)}$ as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively), hence the name.

![PID_controller](https://upload.wikimedia.org/wikipedia/commons/thumb/4/43/PID_en.svg/400px-PID_en.svg.png) 


1. P: Proportional 

   * Term P is proportional to the current value of the SP − PV error ${\displaystyle e(t)}$. For example, if the error is large and positive, the control output will be proportionately large and positive, taking into account the gain factor "Kp". 
   * Using proportional control alone will result in an error between the setpoint and the actual process value because it requires an error to generate the proportional response. If there is no error, there is no corrective response.

> It is the most effective component of the controller as the steeting value is proportional to the error. If we are using P-controller only, we will have an oscillation around the set value. So the P-controller is unstable. 

> As the gain of the P value increase the controller oscillate faster.

2. I: Integral
   * Term I accounts for past values of the SP − PV error and integrates them over time to produce the I term. 
   * For example, if there is a residual SP − PV error after the application of proportional control, the integral term seeks to eliminate the residual error by adding a control effect due to the historic cumulative value of the error. 
   * When the error is eliminated, the integral term will cease to grow. This will result in the proportional effect diminishing as the error decreases, but this is compensated for by the growing integral effect.

> This component adjust for the bias in the sterring angle. This is measured by the sum of the CTE over time.
  
3. D: Differential
   * Term D is a best estimate of the future trend of the SP − PV error, based on its current rate of change. It is sometimes called "anticipatory control", as it is effectively seeking to reduce the effect of the SP − PV error by exerting a control influence generated by the rate of error change. 
   * The more rapid the change, the greater the controlling or damping effect.

> This term solve the issue of oscillation of the P-Controller. This will allow to approach smoothly the target trajectory.

## Tuning
The twiddle algorithm is usually used to further optimize the values of the parameters after I set the initial values.

The main issue that I faced was that the simuator was not working correclty on windows without GPU. So I had to run the code of the workspace of Udacity to get to the correct behaviour. You can read more about it from in [Link1](https://knowledge.udacity.com/questions/281469)

I stared the initial value of the parameter by 
``` Cpp
  Kp = 0.1;
  Ki = 0.004;
  Kd = 3.0;
```

## Maximum Steering angel
The steering value should be within [-1, 1], so I add a small code to fullfill this requirement 
``` cpp
if(steer_value > 1)
{
  steer_value = 1;
}
else if(steer_value < -1)
{
  steer_value = -1;
}
else
{
  /* Do Nothing */
}
```

## Increase the Velocity of the Car
If found that the controller is working fine, so I decided to increase the average velocity to 50mph
```cpp
msgJson["throttle"] = 0.5;
```







# Reference
1. https://en.wikipedia.org/wiki/PID_controller
2. https://www.youtube.com/watch?v=1ImhKwpSmuc
3. https://knowledge.udacity.com/questions/105257

