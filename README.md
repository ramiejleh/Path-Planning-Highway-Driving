# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car is able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Traffic Analysis
Here is a little bit of text about how the problem is approached.
Starting with some traffic analysis using the data from the sensor fusion. looping over the data and to see if there is any car in front of the ego car that is slowing down to less than ego car's speed (which in most cases is max speed). If there is then the ego car matches the car's speed to avoid a collision. After that the other lanes are checked to see if any of them have an empty space. That empty space should be within a safe distance. Available options are marked.
```c++
   // Get car speed
    car_speed = fabs(car_speed);
    // loop over all detected cars from sensor fusion
    for (int i = 0; i < sensor_fusion.size(); ++i){
        // check to see if there are other cars in ego car's lane slower than ego car's speed
        // check if the car is in the same lane
        if ((lane_id*lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < (lane_id+1)*lane_width)) {
            // check if the car is in front is closer than the end of the previous path plus a safety distance             
            if ((sensor_fusion[i][5] < (end_path_s + safety_distance)) && (sensor_fusion[i][5] > (car_s - 5.0))) {
                // calculate other cars absolute speed             
                front_car_speed = std::sqrt(pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) ); 
                const double front_car_speed_in_mph = front_car_speed* 2.23694;
                // calculate the diference in speed between the two cars
                delta_speed = car_speed - front_car_speed_in_mph;
                // if the car is going slower than ego car's speed, match its speed
                if (delta_speed > 0.0){
                    goal_speed = front_car_speed_in_mph;
                    should_match_speed = true;
                } 
            }
        }
        
        // check for cars at left lane in case of switching lanes
        if ((lane_id > 0) && ((lane_id - 1)*lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < lane_id*lane_width)) {
            if ((sensor_fusion[i][5] < (end_path_s + safety_distance)) && (sensor_fusion[i][5] > car_s - safety_distance/2)) { 
                is_left_clear = false;
            }            
        }
        
        // check for cars at right lane in case of switching lanes        
        if ((lane_id < 2) && ((lane_id+1)*lane_width < sensor_fusion[i][6]) && (sensor_fusion[i][6] < (lane_id+2)*lane_width) ) {
            if ((sensor_fusion[i][5] < (end_path_s + safety_distance)) && (sensor_fusion[i][5] > car_s - safety_distance/2)) {
                is_right_clear = false;
            } 
        }            
    }        
              
```

After having the choices marked as valid or invalid, a check is made to see if a lane change is needed. The check consists of determining if there is actually a slower car in front of the ego car and that the ego car is not already in the middle of a lane change. Plus checking if the ego car's speed is less than 90% of the max speed as there is no need to change lanes otherwise. Changing lanes to the left is prefferable.

```c++
    // if the car in front slowed ego car down too much, change lanes if a lane change is not already in progress
    if ((should_match_speed == true) && (change_lane == false)) {
        change_lane = true;
    }
    // Changing lanes and always preferring left
    if(car_speed < (.9 * max_speed) && change_lane == true) {
        if (lane_id > 0 && is_left_clear == true) {
            lane_id = lane_id - 1;
            change_lane = false;
        } else if (lane_id < 2 && is_right_clear == true) {
            lane_id = lane_id + 1;
            change_lane = false;
        } 
    }
```