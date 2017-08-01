# Motion Path Planning - Highway Driving
## Self-Driving Car Engineer Nanodegree Program

The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

The planning algorithm uses states and jerk minimized trajectories in Frenet space.

It requires Udacity's "Term3" version of the simulator, available [here](https://github.com/udacity/self-driving-car-sim/releases).

## Result: [YouTube video: Path Planning](https://www.youtube.com/watch?v=enjdlzo3z2M)

## Structure

The project has four distinct pieces and, unless otherwise noted, where my contribution.
- main.cpp
-- Provided by Udacity to communicate with their simulator
-- Extended with custom getXY() function that works with splines and everything related to the path planner.
- polyTrajectoryGenerator class
-- Everything related to the generation and evaluation of trajectories in Frenet space
- Vehicle class
-- Holds basic position, velocity and acceleration data for ego vehicle and other traffic
- Polynomial class
-- Used to further process jerk minimized trajectories. Contains functions to get three levels of derivatives (down to jerk) at any future timestep.

It also makes heavy use of the spline functions from http://kluge.in-chemnitz.de/opensource/spline/ in spline.h.

## Big Picture
(add decision making graph... feasibility functions, etc)

## Edge case treatment


## Thoughts


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.



## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.




---

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

