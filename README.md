# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Purpose

The purpose of this project is to better understand how a path planning system works.  The code is written in C++ (see below for dependencies).  The project was an excellent learning opportunity and helped me 'dust off' my C++ skills after spending so much time on Python.

**Path planning** provides the autonomous vehicle with trajectories it can use to navigate traffic.  Lane changing is the main function addressed in this project.  **Lane chaning** can be as simple as waiting for traffic to clear and switch lanes but can also be more sophisticated.  For example, the solution provided here _looks forward_ to cars ahead in adjacent lanes to determine if a lane will be blocked ahead.  The solution provided here also looks behind in the lane to avoid switching into a lane with a **speeding car**.  

## Coordinate transforms

The calculations (and hence code) are greatly simplified by using coordinate transforms from Cartesian coodinates to Frenet coordinates and, then, back again to Cartesian coordinates.  Frenet coordinates [www.lepp.cornell.edu/~hoff/LECTURES/10USPAS/notes04.pdf] are useful for making trajectories _from the perspective of the car_.

### Compiling the code

The following scripts are used for compiling the code.

`./clean.sh
./build.sh`

### The code

Much of the logic and code are found in `main.cpp`.  Additional functions and structs are found in helpers.h.  

Important `structs` used are: 
	`struct trajectory_points`, containing the points in the cars trajectory 
    `struct lane_information`, holding inforation about the current and proposed lanes, including speed and position information about the cars ahead and behind the autonomous car being controlled.
    `car_information`,a small struct holding the information about each car in the lane.
    
## Maintaining proper speed and avoiding collisions

On every loop, two flags `too_close` and `too_slow` are evaluated.  If the autonomous car is too_close to the object ahead, it is slowed down by a factor.  If the car is below the speed limit but not too close to another car, then it can speed up.  Too simple functions `slow_down` and `speed up` are provided.  **Note** the code could be improved in several ways with respect to speed control.  Use of a time-based approach was attempted but I was not able to compile the code with _threaded timers_, which would have allowed better control of target speeds and following distances.  The autonomous car doesn't yet take into consideration following distance but does utilize a variable called `safe distance` in order to avoid running into cars ahead.  

## Hyperparameters

Several hyperparameters are used and these can be tuned.  A future version would allow these parameters to be adjusted during program execution.  The parameters are located in ./parameters.csv and are read in `void read_parameters_file()`.  
Useful parameters are: 
    safe_distance (acceptable distance to the car ahead)
    max_speed (the speed limit 49.5; however slower speeds are useful for testing)
    ignore_distance (the program runs more efficiently if cars too far ahead or behind are ignored)
    min_room_ahead (used to determine when there is enough room ahead to make a lane change)
    min_room_behind (used to determine if area behind autonomous car is sufficient for lane change)
    slow_multiplier (factor to reduce the deceleration)
    score_threshold (used to determine if lane change is desirable based on several factors)

## Characterizing Lanes

A major part of the project is charaterizing the lanes.  Cars ahead and cars behind were separated and sorted to find the closest car ahead and the closest car behind.  The speeds of these cars were then stored in a lane_information struct.  In addition, slowest car ahead and total cars ahead were extracted for decision making on the quality of the lane. 

## Scoring possible lanes

The function `double eval_target_lane(vector<car_information> target, vector<car_information> current)` is used to compare the current lane to a proposed target lane using a scoring system. If there is no room available to make a lane change, the score returne is zero. The scoring system was determined through logic and experimentation, incorporating the following:

` score += (target_info.ahead_speed-current_info.ahead_speed);  //closest target faster?
  score += (target_info.ahead-current_info.ahead)/5;  //closest target father ahead?
  score += (ref_vel - target_info.behind_speed)/5;  //going faster than closest target behind
  score += (target_info.slowest_ahead-current_info.slowest_ahead)/5; //slow ahead
  score += (target_info.ahead-target_info.behind)/5; //gap size big
  score += same_lane_slow;
  score -= (target_info.number_of_cars_ahead)*10;
`

In addition, score is incremented in main using `same_lane_slow`, a count of how many frames have been rendered while the autonomous car is stuck in a lane going slower than 40 m/s.  `same_lane_slow` is a measure of how _stale_ the current lane is.

`if ((!target_lane_open) && (ref_vel < 40)) {
      same_lane_slow += .1;
 }
                
if (target_lane_open) {same_lane_slow = 0;}
`

## Deciding to Switch

The following block of code in main.cpp contains part of the logic for choosing the next lane.  This code could certainly be improved in a future version. The following snippet from that block shows the use of the function `eval_target_lane` to return a score for lane 1 when the autonomous vehicle is in lane 0.  For lane 1, the choices are lane 0 or lane 2; therefore, the scores of the two possible lanes are compared.  Note: a lane must have a score > than score_threshold to become the target lane. 

`if (current_lane == 0) {                    
     score = eval_target_lane(lane_data[1],
                              lane_data[current_lane]);
     if (score>score_threshold) {target_lane = 1;}
 } else if (current_lane == 2) {
 '''
`
                 
## Stuck in Lane

The late comedian George Carlin once said 'Anyone who drives slower than you is an idiot and anyone who drives faster than you is a maniac!.'.  Occasionally, the autonmous vehicle would get boxed in.  In order to address this, the following code slows and speeds up the car when it is stuck in a lane going under a threshold speed for longer than a few seconds.  

` //determine if we are stuck; look_back 
  stuck = (same_lane_slow > 1.);  //same lane slow is an accumulator
     if (stuck && look_back) {
        if (follow_distance < 35) {
            follow_distance += .2;
            target_velocity -= .2;
            too_slow = false;
         } else {look_back = false;}  //if greater than 35
     } else if (stuck && !look_back) {  //looking forward
           if (follow_distance > 8) {
               follow_distance -= .2;
               target_velocity += .2;
               too_close = true;
           } else {look_back = true;}
     }
          
     if (stuck) {lane_change_desired = true;}    
          if (!stuck) {  //reset
            follow_distance = safe_distance;
            lane_change_desired = false;
      }
      ...
`

## Web sockets interface.
Some complexity in understanding `main.cpp` comes from its interface to the simulator. For example, the following function call receives incoming data from the simulator: `h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) 
'

### Simulator.
This code does not run on real autonomous vehicles but instead relies on a simulator.  In order to run this code, you must download the Term3 Simulator contining the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)].  
To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Splines

To create smooth trajectories, the spline library was used: http://kluge.in-chemnitz.de/opensource/spline/.

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








