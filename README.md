### Pure Pursuit Controller

# 1. Overview
Use this for pure pursuit algorithm: 
1. https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
2. https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
3. https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9193967

# 2. Algorithm
1. [Algorithm file](https://drive.google.com/file/d/1mGO4xxr3Pa0HzEbteVWy4uK9mj0Gcxyn/view?usp=sharing)
2. ![Algorithm Image](https://github.com/idsc-frazzoli/gokart-core/blob/dev-pure-pursuit-final/packages/motion/control/pure_pursuit_controller/algo.png)

# 3. Steps:

1. Install the repo and checkout to dev-pure-pursuit-final branch. Also go to gokart-ros-base repo and checkout to dev-pure-pursuit.
2. Go to gokart core package and run command `make rebuild-all-local`.
3. Go to `/home/tanmay/eth/sem2/gokart-env-developer/gokart_ws/src/gokart-core/packages/motion/control/pure_pursuit_controller/param` and open `pure_pursuit_params.yaml`. Change the mode in it to the desired mode -
   1. forward - to follow a given path in forward direction
   2. revesre - to follow a given path in reverse direction
    3. deadlock - to implement take home functionality once a test controller fails
4. Run forward mode
    1. Change the mode to `forward` mode in `pure_pursuit_params.yaml`
    2. Generate a path which is an array of Pose2D by any algorithm. Publish this path via the rostopic - `/kitt/path/path_array` of type Path.msg. Start this publisher. If no other algorithm is available, then publish the centreline by going to gokart-core directory and running `make run-center-path-extractor`
    3. Go to gokart-core directory and run `make run-pure-pursuit`.
    4. Go to gokart-core directory and run `make start-pure-pursuit`.
    5. Press the boost button now to run gokart.

5. Run reverse mode
    1. Change the mode to `reverse` mode in `pure_pursuit_params.yaml`
    2. Generate a path which is an array of Pose2D by any algorithm. Publish this path via the rostopic - `/kitt/path/path_array` of type Path.msg. Start this publisher. If no other algorithm is available, then publish the centreline by going to gokart-core directory and running `make run-center-path-extractor`
    3. Go to gokart-core directory and run `make run-pure-pursuit`.
    4. Go to gokart-core directory and run `make start-pure-pursuit`.
    5. Press the boost button now to run gokart.

5. Run deadlock mode
    1. Change the mode to `deadlock` mode in `pure_pursuit_params.yaml`
    2. Generate a path which is an array of Pose2D by any algorithm. Publish this path via the rostopic - `/kitt/path/path_array` of type Path.msg. Start this publisher. If no other algorithm is available, then publish the centreline by going to gokart-core directory and running `make run-center-path-extractor`
    3. Go to gokart-core directory and run `make run-pure-pursuit`.
    4. Start the testing controller which is to be tested and allow it to run. If no testing controller available, we can use the manual mode to run gokart to a deadlock position
    5. Once the gokart leaves the track, apply break to it and stop the testing controller. If no testing controller available, then stop the manual mode if running.
    6. Go to gokart-core directory and run `make start-pure-pursuit`.
    7. Press the boost button now to run gokart in take home functionality mode.

6. To see the progress and debug, open rviz2 and see the markers published by topics - `/kitt/near_point` and `/kitt/goal_point`.


# 4. Nodes

## 4.1 `pure_pursuit_node`
This node runs the pure pursuit controller in the desired mode. It takes input - path to be tracked, gokart state and when to start the controller. It then processes the inputs to find the steering angle and the motors command. It publishes the steering angle to steering controller and publishes motor commands to motors.
### 4.1.1 Parameters
Check file `param/pure_pursuit_params.yaml`
### 4.1.2 Subscribed topics
- `path_array`: [gokart_msgs/msg/Path](https://github.com/idsc-frazzoli/gokart-ros-base/blob/dev-pure-pursuit/packages/gokart_msgs/msg/Path.msg)
- `gokart_state`: [gokart_msgs/msg/GokartState](https://github.com/idsc-frazzoli/gokart-ros-base/blob/dev-pure-pursuit/packages/gokart_msgs/msg/GokartState.msg)
- `start_controller`: [std_msgs/msg/Bool](https://github.com/ros/std_msgs/blob/kinetic-devel/msg/Bool.msg)
### 4.1.3 Published topics
- `steer_ref`: [gokart_msgs/msg/SteerRef](https://github.com/idsc-frazzoli/gokart-ros-base/blob/dev-pure-pursuit/packages/gokart_msgs/msg/SteerRef.msg)
- `near_point`: [visualization_msgs/msg/Marker](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/Marker.msg)
- `goal_point`: [visualization_msgs/msg/Marker](https://github.com/ros2/common_interfaces/blob/master/visualization_msgs/msg/Marker.msg)
- `motors_cmd`: [gokart_cmd_msgs/msg/MotorsCmd](https://github.com/idsc-frazzoli/gokart-ros-base/blob/dev-pure-pursuit/packages/gokart_cmd_msgs/msg/MotorsCmd.msg)

## 4.2 `steering_controller_node`
This node accepts the steer ref angle from `pure_pursuit_node`, implements PID to it and calculates the torque to be published to `steer_cmd`.
### 4.2.1 Parameters
Check file `param/steering_controller_params.yaml`
### 4.2.2 Subscribed topics
- `steer_ref`: [gokart_msgs/msg/SteerRef](https://github.com/idsc-frazzoli/gokart-ros-base/blob/dev-pure-pursuit/packages/gokart_msgs/msg/SteerRef.msg)
- `steer_obs`: [gokart_obs_msgs/msg/SteerObs](https://github.com/idsc-frazzoli/gokart-ros-base/blob/dev-pure-pursuit/packages/gokart_obs_msgs/msg/SteerObs.msg)
### 4.2.3 Published topics
- `steer_cmd`: [gokart_cmd_msgs/msg/SteerCmd](https://github.com/idsc-frazzoli/gokart-ros-base/blob/dev-pure-pursuit/packages/gokart_cmd_msgs/msg/SteerCmd.msg)

## 4.3 `track_node`
This node generates the centre line of default track and publishes it. It is used as default for pure pursuit controller
### 4.3.1 Parameters
Check file `param/pure_pursuit_controller_param.yaml`
### 4.3.2 Published topics
- `path_array`: [gokart_msgs/msg/Path](https://github.com/idsc-frazzoli/gokart-ros-base/blob/dev-pure-pursuit/packages/gokart_msgs/msg/Path.msg)

## 4.4 `start_controller_node`
This node instructs the `pure_pursuit_node` to start running the gokart.
### 4.4.1 Published topics
- `start_controller`: [std_msgs/msg/Bool](https://github.com/ros/std_msgs/blob/kinetic-devel/msg/Bool.msg)



##