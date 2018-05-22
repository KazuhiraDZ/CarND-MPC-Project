# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Abstract
This repository contains the last project of term2. The goal of this project is to navigate a track in simulator, concretely, sending steering and acceleration commands to simulator and let car finish the track with less error(cross track error and orientation).in addition, there is a time latency in real world from getting data to make actuations. The solution should be robust for it.

The solution makes use of IPOPT and CPPAD libraries to calculate an optimal trajectory and related actuations to minimize error with a third-degree polynomial fit to the given waypoints. we should let the MPC only considers a short duration, produces a trajectory for that duration based on a model of vehicle's kinematics and a cost function based mostly on vehicle's cross track error and orientation error, and other factors that may improve performance.

---

## Rubric Points

### The Model
The model contains a 6 parameters' state(xt, yt, psit, vt, ctet, epsit) as input and expects a output (delta, a). The actuators are limited, which contains steering angle(+-25degrees) and throttle(+-1,+1 denotes full throttle and -1 means full breaking).Based on the equations in previous course

```
  xt+1 = (xt + vt * cos(psit) * dt);
  yt+1 = (yt + vt * sin(psit) * dt);
  psit+1 = (psit - vt * deltat / Lf * dt);
  vt+1 = (vt + at * dt);
  ctet+1 = ((ft - yt) + (vt * sin(epsit) * dt));
  epsit+1 - ((psit - psidest) - vt * deltat / Lf * dt);
```

### Timestep Length and Elapsed Duration (N & dt)
N means the total number of points we are considering to calculate the future state of the vehicle. dt is the time interval between the two continuous states. In this implementation, I finally use **N=10** and **dt=0.1** to track a range of **T=1s**. 

#### How to tune the parameters(N and dt)?
Firstly, I use the values same as what I did in course, which is 25 in N and 0.05 in dt. But as what I imagined that the vehicle just rushed to road at begin. In my mine, I think df will reflect the real state if it has a small value, and the nearer the prediction to current state, the more correct output we will get. In addition, larger value of N will cost a lot of computation and we just use some of it, the rest will dropped as we will implement next prediction.

#### How does T(N * dt) affect the prediction?
Maybe increasing N and decreasing dt is a good choice to increase the accuracy of prediction. But if the car at high speed, we have large N and very small dt, then the computation will take a lot of time that the vehicle can't have actuation in time.

### Polynomial Fitting and MPC Preprocessing
Compare to MPC quiz, this project we need convert the global coordinates ptsx and ptsy to vehicle's local coordinate **waypoints_x** and **waypoints_y**. Then I use polyfit on waypoints to get coefficients of 3rd-degree curve in the line the car will be moving. Based on these coefficients I can compute the cross track error and orientation error, both of them will be used to next prediction.


### Model Predictive Control with Latency
In my implement, I take the latency into account that I use it to compute a future state as my current state, then take it in my MPC model to predict, which you can find at line 110-122 in **main.cpp**


---
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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
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
