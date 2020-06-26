Assignment 4 - Path Following
======

DESCRIPTION
------
This package utilises a robot simulation within a world. A path of goals is set and the robot tries to reach each goal sequentially.
If the selected goal is obstructed, it will be removed and the next goal will be assessed. 
Assessment occurs at each pose before any movement.

This package consists of three nodes:
- Path Manager
- Velocity Controller
- OGMap Visualiser

Nodes within this package are designed to be modular as specified in [OOP with Ros in CPP](https://roboticsbackend.com/oop-with-ros-in-cpp/).

#### Path Manager
The Path Manager Node listens to input from rviz (2D Nav Goal), and publishes the poseArray msg to the /raw_data topic.

It also subscribes to /raw_path and determines if the next goal is obstructed. If not obstructed, the path is published to /robot_0/path.

#### Velocity Controller
The velocity Controller node subscribes to various topics including robot_0/path, alters the linear and angular velocity and publishes Twist messages to /robot_0/cmd_vel.
Obstacles will be avoided during pursuit. If evasive action is required the current goal is abandonded, and the resulting path is published back to /raw_path for assessment.
If Path Manager is not used, the robot will immidiately pursure the next goal (obstructed or not).
Once the goal has been reached, the goal is removed from the /robot_0/path and published to /robot_0/completed_path.

#### OGMap Visualiser
This node subscribes to robot_0/path, and /robot_0/completed_path and draws red, green or blue circles on the OGMap image. It then publishes the image to the
/map_image/path_following topic. 

## Operational Description:
This package utilises rviz to select a series of poses (position and orientation using 2D Nav Goal). The user has 5 seconds to select a goal after the last input. 
If no input is detected after 5 seconds, the path will be published to robot_0/path. Rviz has also been used to display the OGMap and points.
After a goal has been identified and is reachable, the robot will rotate on spot to match the direction of the goal. It will then travel to the goal point using either point 
and shoot, or pure pursuit (depending on configuration selected in launch file). Once the goal point is reached, the robot will again rotate on spot to match 
the goal orientation. It will then immidiately assess the next pose and restart the movement process.
If an obstacle is detected during pursuit, the robot will reverse, then abandon it's current goal and then pursue the next goal.

The OGMap displays completed goals in blue, current goal in green, and future goals in red. The black square in the centre of the image (100,100), represents the robot.


## Deployment:
Start roscore:  
```bash
roscore
```

Start the world simulation:
```bash
roslaunch a4_setup a4_setup.launch 
```

By default, Pure Pursuit is disabled. 
If you wish to use Pure Pursuit, edit the launch file:
```
<arg name="pure_pursuit" default="true"/>
``` 

For ease of use, Path Manager, Velocity Controller, OGMap Visualiser, and rviz can be launched by running:
```
roslaunch a4_path_following a4_path_following.launch 
```

#### Nodes
To run the nodes individually, run the following commands:
 ```
 rosrun rosrun a4_path_following a4_path_following-path_manager
 ``` 

 ```
 rosrun rosrun a4_path_following a4_path_following-velocity_control
 ``` 

 ```
 rosrun rosrun a4_path_following a4_path_following-ogmap_visualiser
 ``` 

## Pure Pursuit
Optionally, to enable Pure Pursuit via command line, the following commands can be used:

Launch File:
```
roslaunch a4_path_following a4_path_following.launch pure_pursuit:=true 
```

Node:
```
 rosrun a4_path_following a4_path_following-velocity_control _pure_pursuit:=true 
 ```

## RQT graph
[RQT Graph](docs/rosgraph.png)




