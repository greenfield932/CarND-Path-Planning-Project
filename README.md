# CarND-Path-Planning-Project

[image1]: ./images/1-3.gif
[image4]: ./images/4.gif
[image5]: ./images/5.gif
[image6]: ./images/6.gif
[image7]: ./images/model1.png
[image8]: ./images/model2.png
[image9]: ./images/model3.png

### Goals
The main goal of the project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost 
as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, 
it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Input data
Input data include vehicle position values (x, y, s, d, yaw) and speed values. Surronding environment represented by a way points of the middle of the track, lane count, lane width, and sensor fusion data 
(a list of cars present on the track and their kinematic and position parasmeters).

### Criteria proof
1. The car is able to drive at least 4.32 miles without incident
2. The car drives according to the speed limit.
3. Max Acceleration and Jerk are not Exceeded.

The following video represents a car driving over 4.32 miles without incident, speed lower than 50 mph limit and no acceleration or jerk exceeded during the test.
![alt text][image1]

4. Car does not have collisions.

The following video represents an ability to handle collisions. When the car became to close to a slow vehicle the car's speed adjusted according to the vehicle ahead speed.

![alt text][image4]

5. The car stays in its lane, except for the time between changing lanes.

The following video represents the ability of the car to keep lane on a slightly curved road.

![alt text][image5]

6. The car is able to change lanes

The following video represents the ability of the car to change lanes.

![alt text][image6]

## Implementation details on trajectory generation

1. Calculate middle points at specified coordinates si based on the track waypoints and their corresponding representation on the vehicle’s lane 
[vehicle.cpp lines: 90-134](src/vehicle.cpp#L90)

![alt text][image7]

2. Convert Frenet s,d coordinates of the points from steps1 to global x,y coordinates (vehicle.cpp lines: 90-134)
3. Convert x,y coordinates to vehicle’s coordinates x’, y’ relative to reference point (vehicle’s position) 
(shifting all points relative to reference point and [apply rotation](https://en.wikipedia.org/wiki/Rotation_matrix) with vehicle’s yaw angle) 
4. Make a [spline](http://kluge.in-chemnitz.de/opensource/spline) F(x’) in the vehicle’s coordinate system using points from step3 

![alt text][image8]

5. Using F(x’) generate trajectory points using dx calculated using refresh rate dt, horizon and desired speed. 
6. Convert trajectory points back to global coordinates from map coordinates

![alt text][image9]

## Implementation details on behavior planning

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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