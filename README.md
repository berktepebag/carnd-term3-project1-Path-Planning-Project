### *** Archieved: As this repo is and will not updated anymore, it is archieved. ***

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program Term 3 Project 1

#### System: Dell XPS 13 - Windows 10 Ubuntu Bash
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Calculating Next x,y Steps

To calculate next x,y steps we will use polynomials. Since the x,y and yaw angle of the car received from the simulator, it is easy to predict a route which the car will take in next few steps. There are two conditions: there is info about last steps and the car just started moving.  If last steps exists, while fitting polynomial they can be used to make a better prediction. If car just started moving, create dummy last x,y variables from current x,y and yaw angle variables to smoothen the path predicted.

## Creating Spline Points

spline.h is a good one header file which can easily calculate polynomials and can be used with previously calculated x,y steps. 
   tk::spline s;
   s.set_points(ptsx,ptsy,true); will return the polynomial. Gather previous_path_x and previous_path_y  information from the simulator and fill next_x_vals and next_y_vals with it. For the amount of steps missing in next_x/y_vals add from newly calculated spline points. 
   
## Lane Cost Functions

The system is developed on lane costs. Variables are, 
1. Distance between the ego car and the other vehicle in control distance (and if the other vehicle is in close range, s < 5m and d < 5m double the cost)
2. Speed of the other vehicle
3. Total cost of a vehicle = distance / speed

#### Fast&Further!

Since having 3 fast cars in a lane is better than having a slow car in a lane, our algorithm favors vehicles which are faster and further. That is why distance is divided by the speed of the vehicle.

## Counting the vehicles

Since highways are crowded most of the time, sensor fusion data can also be crowded. Since we do not need all the vehicles on the road, we must decide where to look. It will look for 100 meters ahead and 10 meters back.

<img width="800" alt="Counting other vehicles" src="/imgs/counting_cars.jpg">

## Vehicles in the lanes

Since knowing where the other vehicles positions is crucial also knowing their lane will helps us changing lanes safely.

## Decreasing the lane costs over time

If lane_costs vector created outside of the loop and is set zero for each time, calculated costs will be exact moments costs. The logic will then tell the car to change lane according to minimum lane cost, which causes jerk. Instead for every frame lane costs are summed with new lane costs and for every 10 frames divided by 2. So if there is no vehicle left in the lane, the ego car would not suddenly change lane but will wait until empty lane reaches smallest lane cost.   

## Deciding best lane according to the costs

A for loop handles the finding minimum lane cost. Another for loop will handle the situation according to min-lane-cost and safety of the intended lane. If there is no car in 20 meters range the car will move to the lane. Else stays and starts calculating the costs again.

<img width="800" alt="Counting other vehicles" src="/imgs/finding_the_best_lane.JPG">

## Jerk and Acceleration

Sudden lane changes creates high jerks and acceleration which is not comfortable passengers. To prevent this and if statement checks if time (counter) has come and speed is lower than 37.0 mph. Higher speeds causes a trigger in jerk/accelearation. 

## Distance with the vehicle

If the distance between ego car and the front vehicle is less than 30 meters, ego car will slow down with an ratio of 8/(distance between two vehicles) and will keep the distance.

## Known problems

1. Some turns causes s and d to be mis-calculated which creates problems with following the vehicle in front. Since using radar and UKF will fix this problem, it will be accepted as known and not worth fixing since it happens very few times. 

## Car in Action


<a href="https://www.youtube.com/watch?v=WrDXzFpHefw&feature=youtu.be" target="_blank"> 10 Miles Run Without Incident </a>

