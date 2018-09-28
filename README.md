
# Path Planning Project


## INTRODUCTION 

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

----

### Images

<img src = 'img/1.png' width = "500"/>
<img src = 'img/2.png' width = "500"/>
<img src = 'img/3.png' width = "500"/>



## Working

there are two main parts of the project

----

#### Prediction and Decision making 


##### initialize flag to specific car around us on start


----


```
 // avoid collitions
                                if (prev_size > 0) {
                                    car_s = end_path_s;
                                }

                                // other cars positions.
                                bool car_ahead = false;
                                bool car_left = false;
                                bool car_righ = false;
```

##### Sensor fusion give us data of the car present around us

```
                                //sensor_fusion giving us data of the cars present around us
                                double sensorFusionSize=sensor_fusion.size();
```
##### d is the frenet coordinate 
##### In below code we try to predict the in which lane is the car
##### we are also trying get mangnitued of vector by using x and y veocity 
##### getting estimate position of car via this code double check_car_s = sensor_fusion[i][5];


##### and getting s value outwards, kind of what car will look like in future  using this code 
check_car_s += ((double)prev_size*0.02*check_speed);
```
                                for ( int i = 0; i < sensorFusionSize; i++ ) {
                                    float d = sensor_fusion[i][6];
                                    int car_lane = -1;
                                    // is it on the same lane we are
                                    if ( d > 0 && d < 4 ) {
                                        car_lane = 0;
                                    } else if ( d > 4 && d < 8 ) {
                                        car_lane = 1;
                                    } else if ( d > 8 && d < 12 ) {
                                        car_lane = 2;
                                    }
                                    if (car_lane < 0) {
                                        continue;
                                    }
                                    //getting the speed of the car
                                    double vx = sensor_fusion[i][3];
                                    double vy = sensor_fusion[i][4];
                                    // getting x and y velocity getting magnitued
                                    double check_speed = sqrt(vx*vx + vy*vy);
                                    double check_car_s = sensor_fusion[i][5]; // this is s value of car to chek if it reallly close to us or not
                                    // gettting estimate of other car position
                                    check_car_s += ((double)prev_size*0.02*check_speed);
```
##### change if anoterh cars are at left or right or ahead of us then taking action accordingly 
```
                                    

                                    int gap = 30;
                                    if ( car_lane == lane ) {
                                        
                                        
                                        car_ahead |= check_car_s > car_s && check_car_s - car_s < gap;
                                    } else if ( car_lane - lane == -1 ) {
                                        // car at left
                                        car_left |= car_s - gap < check_car_s && car_s + gap > check_car_s;
                                    } else if ( car_lane - lane == 1 ) {
                                        // car at right
                                        car_righ |= car_s - gap < check_car_s && car_s + gap > check_car_s;
                                    }
                                }

```
##### This code is to take action if all flags are true and change spped of the car according to the realtive speed 

##### so here all the decisions are made according to the environment  
```
         double speed_diff = 0;
                                const double MAX_SPEED = 49.5;
                                const double MAX_ACC = .224;
                                if ( car_ahead ) { // Car ahead
                                    if ( !car_left && lane > 0 ) {
                                        
                                        lane--;
                                        // Change lane to left
                                    } else if ( !car_righ && lane != 2 ){
                                        
                                        lane++;
                                        // Change lane to right.
                                    } else {
                                        speed_diff =speed_diff - MAX_ACC;
                                    }
                                } else {
                                    if ( lane != 1 ) { // if we are not on the center lane.
                                        if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) {
                                            lane = 1; // Back to center.
                                        }
                                    }
                                    if ( ref_vel < MAX_SPEED ) {
                                        speed_diff =speed_diff + MAX_ACC;
                                    }
                                }
```
-----
#### Trajectory Generation 

##### The below code helps in trajectory generation 

##### Initialise last couple of points so that car can move in certain angele of previous points
##### and we have the location of car in 30 , 60 and 90 m away 




```
                                vector<double> ptsx;
                                vector<double> ptsy;
                                
                                double ref_x = car_x;
                                double ref_y = car_y;
                                double ref_yaw = deg2rad(car_yaw);
                                
                                if ( prev_size < 2 ) {
                                    double prev_car_x = car_x - cos(car_yaw);
                                    double prev_car_y = car_y - sin(car_yaw);
                                    
                                    ptsx.push_back(prev_car_x);
                                    ptsx.push_back(car_x);
                                    
                                    ptsy.push_back(prev_car_y);
                                    ptsy.push_back(car_y);
                                } else {
                                    ref_x = previous_path_x[prev_size - 1];
                                    ref_y = previous_path_y[prev_size - 1];
                                    
                                    double ref_x_prev = previous_path_x[prev_size - 2];
                                    double ref_y_prev = previous_path_y[prev_size - 2];
                                    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
                                    
                                    ptsx.push_back(ref_x_prev);
                                    ptsx.push_back(ref_x);
                                    
                                    ptsy.push_back(ref_y_prev);
                                    ptsy.push_back(ref_y);
                                }
                                int gap = 30;
                                // Setting up target points in the future.
                                vector<double> next_wp0 = getXY(car_s + (gap), 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                vector<double> next_wp1 = getXY(car_s + (gap*2), 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                vector<double> next_wp2 = getXY(car_s + (gap*3), 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                                
                                ptsx.push_back(next_wp0[0]);
                                ptsx.push_back(next_wp1[0]);
                                ptsx.push_back(next_wp2[0]);
                                
                                ptsy.push_back(next_wp0[1]);
                                ptsy.push_back(next_wp1[1]);
                                ptsy.push_back(next_wp2[1]);
                                
                                // creating local car coordinates.
                                for ( int i = 0; i < ptsx.size(); i++ ) {
                                    double shift_x = ptsx[i] - ref_x;
                                    double shift_y = ptsy[i] - ref_y;
// changing car reference angle for our own ease

                                    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                                    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                                }


```
##### We can use Polynomial fit instead of spilne maintly help to find the path of the car

##### adding points from previous path to path planner , help in the transition 
##### i.e spinle is used to smoothen the trajectory 

```

tk::spline s;
                                s.set_points(ptsx, ptsy);
                                
                                // Output path points from previous path for continuity.
                                vector<double> next_x_vals;
                                vector<double> next_y_vals;
                                for ( int i = 0; i < prev_size; i++ ) {
                                    next_x_vals.push_back(previous_path_x[i]);
                                    next_y_vals.push_back(previous_path_y[i]);
                                }
                                
                                // Calculate distance y position on 30 m ahead.
                                double target_x = 30.0;
                                double target_y = s(target_x);
                                double target_dist = sqrt(target_x*target_x + target_y*target_y);
                                
                                double x_add_on = 0;
                                
                                for( int i = 1; i < 50 - prev_size; i++ ) {
                                    ref_vel += speed_diff;
                                    if ( ref_vel > MAX_SPEED ) {
                                        ref_vel = MAX_SPEED;
                                    } else if ( ref_vel < MAX_ACC ) {
                                        ref_vel = MAX_ACC;
                                    }
                                    double N = target_dist/(0.02*ref_vel/2.24);
                                    double x_point = x_add_on + target_x/N;
                                    double y_point = s(x_point);
                                    
                                    x_add_on = x_point;
                                    
                                    double x_ref = x_point;
                                    double y_ref = y_point;
                                    
                                    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                                    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
                                    
                                    x_point += ref_x;
                                    y_point += ref_y;
                                    
                                    next_x_vals.push_back(x_point);
                                    next_y_vals.push_back(y_point);
                                }
                                
```                               
