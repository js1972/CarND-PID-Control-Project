# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Intro
The purpose of this project is to create a PID controller to drive a car smoothly and successfully around the Udacity simulator.

## Reflection
The PID controller is made up fo 3 components:<br>
P - Proportional: Here we steer the car proportionally to its offset from the center of the road (or desired path). An issue with just using a proportional controller is overshoot.

[![PID controller with only P component active](https://img.youtube.com/vi/OU4RHUCBLJg/0.jpg)](https://www.youtube.com/watch?v=OU4RHUCBLJg)

D - Differential: He we add some feedback into the contoller about how quickly it is moving back to the required path and essentially dampen this movement.

[![PID controller with only P component active](https://img.youtube.com/vi/PhVvHx5NFxw/0.jpg)](https://www.youtube.com/watch?v=PhVvHx5NFxw)

I - Integration: He we integrate (sum) over the error and feed this signal back into our controller to compensate for any drfit in the vehicle. Drift can occur for variour reasons such as misalignments of the wheels for example. In this project I found zero or very close to zero drift in the vehicle. My best parameters have the I coeffecicnt set to 0.001. See the video below using the best parameters found with the twiddle algorithm explained further below...

[![PID controller with only P component active](https://img.youtube.com/vi/B_f4edzNUuw/0.jpg)](https://www.youtube.com/watch?v=B_f4edzNUuw)

---

### Tuning
I used the "Twiddle" algorithm to tune the coefficient for the P, I and D components of the controller. To run the controller in twiddle mode simply anter true as a parameter, i.e. `./pid true` or `./run.sh true`.

What the twiddle algorithm does is continually adjust the PID coefficients as the car drives, trying to find the best (or lowset) overall error.
I found the following parameters which are set in the PID initialisation as ones that provided a nice smooth driving experience around the track.
```
pid.Init(0.15, 0.001, 1.5);
```

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/aca605f8-8219-465d-9c5d-ca72c699561d/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
