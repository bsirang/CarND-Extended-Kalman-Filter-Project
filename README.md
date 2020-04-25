# Extended Kalman Filter Project

C++ implementation of an Extended Kalman Filter, fusing LIDAR and Radar sensors together to estimate a moving object's state.

This filter is designed to work with Udacity's CarND Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## State Vector
This EKF implementation uses a 4-dimensional state vector composed of 2D position and 2D velocity.

## Simulator Protocol

To communicate with the simulator we're using `uWebSocketIO`.

### Inputs

**["sensor_measurement"]** => the measurement that the simulator observed (either lidar or radar) as well as the ground truth

### Outputs

**["estimate_x"]** <= kalman filter estimated position x

**["estimate_y"]** <= kalman filter estimated position y

**["rmse_x"]** <= x position root mean squared error with respect to ground truth

**["rmse_y"]** <= y position root mean squared error with respect to ground truth

**["rmse_vx"]** <= x velocity root mean squared error with respect to ground truth

**["rmse_vy"]** <= y velocity root mean squared error with respect to ground truth

## Build and Run Instructions

This has been tested on Linux, but should be able to run on OS X and Windows as well.

1. First run `install-linux.sh` or `install-mac.sh` to install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) and its dependencies.
1. `mkdir build`
1. `cd build`
1. `cmake ..`
1. `make`
1. `./ExtendedKF`

At this point, you can start the simulator.

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
