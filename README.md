# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
---

# Reflection

## how to generate paths.

### the simulator

The code is run with the simulator. The simulator reads the list of path waypoints and moves the ego car according to the points every 0.02s and then returns the previous waypoints left and the current ego car states. The tricky part is that some waypoints in the previous path are comsumed even when the onMessage in the main code is running. So I need to use the previous waypoints and predict the path from the returned ego car state plus some timesteps. 

These comsumed timesteps were one to five waypoints. I decided to safely predict the path from the last waypoints, end_path_s in the previous path.


### coordinate

<img src="./map.png" width=500/>


The generated paths of the prediction model are in Frenet coordinates because it depends on d, lane position. But the simulator reads way points in Cartesian coordinates so I need to convert s and d to x and y. The s and d in the Frenet coordinates are in the linear segments approximating the x and y road. The converted x and y from the s and d, therefore, has the rapid change of the angle in each node connecting the linear segments and it produce the large value of  acceleration and jerk even thought you generated constrained s and d waypoints.


### prediction
The state of ego car and other vehicles should be predicted for path generation. It is assumed that they have a constant acceleration and predicted from sensor fusion data.
``` vehicle.cpp
double vehicle_s = v.s + ((double)path_size * Timestep * v.v);
```

### behavior planning
It first checks available next lanes. The highest speed lane of them is selected for the lane of the next state. 
The state machine has the states of {KL, PLCR, PLCL, LCR, LCL}. The main transition criteria are target_lane and lane like following. 


``` behavior.cpp
string next_state;
if (state.compare("KL") == 0) {
	target_lane = available_next_lane;

	if (target_lane < lane)
			next_state = "PLCL";
	else if (target_lane > lane)
			next_state = "PLCR";
	else
			next_state = "KL";

} else if (state.compare("PLCL") == 0) {
	if (collision_list[target_lane]) {
			next_state = "PLCL";
	} else {
			next_state = "LCL";
	}
} else if (state.compare("PLCR") == 0) {
	if (collision_list[target_lane]) {
			next_state = "PLCR";
	} else {
			next_state = "LCR";
	}
} else if (state.compare("LCL") == 0) {
		lane = target_lane;
		next_state = "KL";
} else if (state.compare("LCR") == 0) {
		lane = target_lane;
		next_state = "KL";
}

``` 
It adjusts the speed according to the lane traffic. It uses 0.1m/s increment. It compares speed with front vehicle except in urgent case, like short buffer, 

```
double next_v = this->target_speed;
if (collision_list[lane]) {
	cout << "reducing speed=" << road.ego.v
				<< "... at ego lane according to v=" << collision_vehicles[lane].v << endl;
	double buffer = collision_vehicles[lane].s - end_path_s;
	if (collision_vehicles[lane].v < next_v
			|| (buffer > 0 && buffer < 15)) {

			if (fabs(buffer) < 5) {
					cout << "COLLISION at v=" << road.ego.v << ", buffer="
								<< buffer
								<< ", lane=" << lane << endl;
			}
			next_v -= 0.1;
			if (next_v < 1)
				next_v = 1;
	}
}
else {
	next_v += 0.1;
	if (next_v > Speed_Limit)
		next_v = Speed_Limit;
}
this->target_speed = next_v;
```

### trajectory generation
It need to be smoothy to be content with max acceleration and jerk constraints. I frist used JMT method. Despite of best effort, it was not possible to satisfy the constraints. So I returned the method shown in Walkthrough video.

That method generates smoothy reference trajectory which is heading the target lane. The reference trajactory is used to measure the distance satisifying the constraints.

I inserted 30, 60, 90m waypoints fram the car to generate spline and found that in some curcumstances it violates the constraints. It is because the generated trajectory is rapid curved. I changed the waypoints to insert to 60, 90m away from the car. It made more smoothy waypoints within constraints. It also need to insert some more waypoints from previous path.

``` main.cpp
for (int i = 2; i <= 3 ; i ++) {
	vector<double> next_point = getXY(car_s + 30 * i
																		, lane2d(behavior.lane)
						, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	ptsx.push_back(next_point[0]);
	ptsy.push_back(next_point[1]);
}

```

   
## future work
It will be better to enhance available lane selection. It chooses only left and right lane of the current lane. Even busy next lane could be selected to go far free lane.








---   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

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


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
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

