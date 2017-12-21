# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
---

# Reflection

## how to generate paths.

### the simulator

The code is run with the simulator. The simulator reads the list of path waypoints and moves the ego car according to the points every 0.02s and then returns the previous waypoints left and the current ego car states. The tricky point is that some waypoints in the previous path are comsumed even when the onMessage in the main code is running. So I need to use the previous waypoints and predict the path from the returned ego car state plus some timesteps. This comsumed timesteps was one to five waypoints. I decided to safely predict the path from the last waypoints in the previous path.

```
for (j = 0; j < path_size; j++)
{
    //cout << "[" << previous_path[j].s << ", " << previous_path[j].d << "<" << previous_path[j].intended_lane << ">, " 
    //            << previous_path[j].v << ", " << previous_path[j].a << ", theta=" 
    //            << previous_path[j].theta << ", psi=" << previous_path[j].psi << ", delta=" << previous_path[j].delta << "], ";

    next_x_vals.push_back(previous_path_x[j]);
    next_y_vals.push_back(previous_path_y[j]);

    spline_x.push_back(previous_path_x[j]);
    spline_y.push_back(previous_path_y[j]);
}
```          

### coordinate

![map](./CarND-Path-Planning-Project/map.png)
The returned waypoints are in previous_path_x and previous_path_y. It will be better to preserve more states so defined previous_path which is vector<Vehicle>. Vehicle has s and d in Frenet coordinate and v, total velocity and a, total acceleration of the car. The angle of x and y is theta and that of s and d is psi and the angle of x axis and d vector is delta. 

```
for(int i = 1; i <= horizon - path_size; i++)
{
      traj[i].psi = atan2((traj[i].s - traj[i - 1].s), (traj[i].d - traj[i - 1].d));
      if (traj[i].s > Max_S) {
        traj[i].s -= Max_S;
      }

      way_s = traj[i].s;
      way_d = traj[i].d;
      xy = getXY(way_s, way_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

      traj[i].x = xy[0];
      traj[i].y = xy[1];
      auto cw = ClosestWaypoint(xy[0], xy[1], map_waypoints_x, map_waypoints_y);
      double delta = atan2(map_waypoints_dy[cw], map_waypoints_dx[cw]);
      traj[i].theta = atan2((traj[i].y - traj[i - 1].y), (traj[i].x - traj[i - 1].x));
      traj[i].delta = delta;
      //cout << "[" << way_s << ", " << way_d << ", theta=" << traj[i].theta << ", psi=" 
      //    << traj[i].psi << ", delta=" << traj[i].delta << "(" << cw << ")], ";

      spline_x.push_back(traj[i].x);
      spline_y.push_back(traj[i].y);
}

```

The generated paths of the prediction model are in Frenet coordinates because it depends on d, lane position. But the simulator reads way points in Cartesian coordinates so I need to convert s and d to x and y. The s and d in the Frenet coordinates are in the linear segments approximating the x and y road. The converted x and y from the s and d, therefore, has the rapid change of the angle in each node connecting the linear segments and it produce the large value of  acceleration and jerk even thought you generated constrained s and d waypoints.

It need to be smoothy to be content with max acceleration and jerk constraints. I used two spline, timestep to s and timestep to d because the spline library only accept monotonically increasing dependent variable.

```
spline_t = {0
      , 1
      , (double)(spline_y.size() - 1)};
spline_x2 = {spline_x[0]
      , spline_x[1]
      , spline_x[spline_x.size()-1]};
spline_y2 = {spline_y[0]
      , spline_y[1]
      , spline_y[spline_y.size()-1]};

sx.set_points(spline_t, spline_x2);
sy.set_points(spline_t, spline_y2);
```


### prediction
The state of ego car and other vehicles should be predicted for path generation. It is assumed that they have a constant acceleration and predicted from Vehicle::state_in() and Road::populate_traffic() and sensor fusion data.
```
vector<double> Vehicle::state_in(double t) const {
  double s = this->s + this->v*t + 1/2*this->a*t*t;
  double v = this->v + this->a*t;
  double a = this->a;
  return {s, v, a};
}
```

### behavior planning
The state machine has the states of {KL, PLCR, PLCL, LCR, LCL}. I designed to generate paths of each states and choose the path with the lowest cost and transition according to the state of the path. If it tries to do lane change actions in a timestep in some curved lane, it will produce large acceleration and jerk beyond the constrains. It can conduct the lane change through several timesteps, the state machine have LCL to LCL, and LCR to LCR also. "if (this->intended_lane != this->lane)" means it is in the middle of the lane change. intended_lane is set to +/-1 with lane number only when it starts lane changing.
```
} else if (state.compare("LCR") == 0) {

  if (this->lane == lanes_available - 1)
    states.push_back("KL");
  else {

    if (this->intended_lane != this->lane)
      states.push_back("LCR");
    else {
      states.push_back("KL");
    }
  }
}
``` 

I tried several cost functions but the buffer_cost function is enough to the project. It compares the current lane's buffer lenght to the intended lane's.


### trajectory generation
At each timestep, it predicts the position, velocity and acceleration of the ego car and the other vehicles and generates suitable trajectories of each states from state machine avoiding collision. Finally it choose the lowerest trajectory of them. The hardest problem was generating s and d values satisfying constraints. I think it will be easier if you choose some increment value of position and advance the ego car and comparing the calculated velocity and acceleration with the constraints. But I tried to do JMT method. It garentees that it makes the path jerk minimized but it is hard to contrain the jerk to some value. I used this value for jmt_s = JMT({s0, v0_s, a0_s}, {s, v_s, a_s}, Ts)

```
    double Ts = Timestep*horizon;
    double s0 = this->s;
    double v0_s = this->v*sin(this->psi);
    double a0_s = this->a*sin(this->psi);

    double v_s = this->target_speed*sin(this->psi);
    double s = s0 + v0_s * Ts;
    double a_s = a0_s;
```
It was good except at the start time. It gave max acceleration or max jerk error at start time. I adjusted the parameter for slow start.

```
    bool slow_start = false;
    if (v0_s <= 15 && v_s > 15) {
      v_s = v0_s + 0.005;
      s = s0 + v_s * Ts;
      slow_start = true;
    }
```

I used the following code for jmt_d = JMT({d0, v0_d, a0_d}, {d, v_d, a_d}, Td). When it starts, the psi value is not well defined. The wrong psi value makes the car trun. So I fixed it at the start time. As I told, the lane changing actions makes max accererlation or max jerk sometimes if it is done in a timestep. So I divided it by 6 and made it done through six timesteps.

```
    double d0 = this->d;
    double v0_d = this->v*cos(this->psi);
    double a0_d = this->a*cos(this->psi);

    double d;
    if (fabs(this->lane - this->intended_lane) != 0) {
      d = (lane2d(this->intended_lane) - lane2d(this->lane)) / 6 + d0;
    } else
      d = lane2d(this->intended_lane);
    double v_d = v0_d; 
    double a_d = a0_d;
    double Td = Timestep;
    if (slow_start) {
      d = d0;
    }
```
   
## future work
The code made the car run without incident 19.15miles at most. It sometime fails to avoid abrupt collison. It needs to enhance JMT with constraints for adjusting speed change rapidly.








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

