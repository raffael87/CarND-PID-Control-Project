# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Goal
Goal of this project is to implement a pid controller which helps the car to steer and in best case also to adapt the velocity.
The simulation publishes in each cycle the CTR (cross track error), steering angle and the speed of the vehicle.
With this data, the PID controller should manoeuvre the car safely.

## Projects Specification
### Describe the effect each of the P, I, D components had in your implementation.
* The proportional component has a big impact on the vehicles steering. With the P component the proportion of the steering can be controlled. If the distance of the vehicle to the lane center is big, then it will counter steer stronger. If it is closer to the center lane (smaller CTR), it will counter steer less

* The differential component helps to avoid that the vehicle overshoots the center line. Without this component, the car would overshoot the line, counter steer, overshoot again etc. Having an optimal parameter value makes the vehicle approach the center line, but do not overshoot

* The integral component helps to minimize a bias which the CTE can have (e.g. steering wheels are not aligned properly). Having this bias, it can happen that the normal PD controller is not able to steer towards the center line.

* During tuning of the hyperparameters it is quite visible how bigger changes affect the steering. This can be seen here, where a p value from 6 is chosen:
[![IMAGE ALT TEXT](http://img.youtube.com/vi/Nmo-DLA9YoA/0.jpg)](http://www.youtube.com/watch?v=Nmo-DLA9YoA"Driving "bad P")

### Describe how the final hyperparameters were chosen.

1. PID class was implemented in order to have a running controller. Controller was connected with the simulation.
2. The first hyperparameters where chosen via trial and error. The goal was to have parameters where the vehicle could drive a lap without crashing. The throttle was constant with 0.3
3. The twittle component was implemented. It follows the structure from the lesson. The component updates after each cycle and a period of settling in the error. If the car crashes or a lap is performed, then the parameters are being updated and the simulation is restarted.
4. In order to learn the hyper parameters a few iterations (laps) have to be done.
5. In the end a the hyperparameters were trained and a good result was achieved with: `P: 0.124, I: 0.00027, D: 3.03`
6. Speed and throttle were also connected to a pid controller, but the results where not that optimum. A training of the hyperparameters was not performed yet

Result:
[![IMAGE ALT TEXT](http://img.youtube.com/vi/9liJ9HQpgMY/0.jpg)](http://www.youtube.com/watch?v=9liJ9HQpgMY "Driving PID")

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
