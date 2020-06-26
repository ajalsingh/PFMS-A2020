Quiz 8
======

Part A
------
You have been provided piece of code your colleauges have developed for Assignment 1/2, and we will be debugging these pieces of code.

![Debugging Pain](https://media.giphy.com/media/6yRVg0HWzgS88/giphy.gif)

For questions 1-3 use [vector_ops.cpp](./a/vector_ops.cpp). The intent of this code was to create a STL container resembling a matrix of elements (2x5).
 
The code compiles (no compile-time error) and also runs (no run-time errors).    
Yet, it fails to display the values (you would anticipate to see two rows of values).
```
0 1 2 3 4
0 1 2 3 4
```
The code has a few points of failure causing unintended behaviour.

For questions 4-5 use [main.cpp](./a/main.cpp) and the [Shape](./a/shape.h), [Rectangle](./a/rectangle.h) and [RectangleHelper](./a/rectanglehelper.h) class.

The code compiles (no compile-time error) and also runs (no run-time errors). Yet, it fails to determine line intersects.
Further, the RectangleHelper class that was supposed to print properties seems to be not working either.

1) QUESTION: When you request a change in capacity [reserve keyword](http://www.cplusplus.com/reference/vector/vector/reserve/), does that create elements of the vector.
    - No. It has no effect on vector size and cannot alter elements.

2) QUESTION: What line of code is the first point of failure that could have been detected if the STL container was accessed correctly.
    - Reserve does not create elements in the vector. Row vectors must first be pushed back before pushing back doubles

3) QUESTION: Once errors associated with creating / storing data into the matrix are fixed, what additional error in this code results in incorrect matrix size.
    - Use less than (<) instead of less and equal to (<=).
    - The following code:
        ```c++
        for (unsigned int idx=0;idx<=cols;idx++){}
        ```
    - Should be:
        
        ```c++
        for (unsigned int idx=0;idx<cols;idx++){
        ```

4) QUESTION: Why does the intercept method in `Rectangle` fail to report the intercept correctly?
    - the printInterceptCheck function creates a new instance of Rectangle with default params. The instance should be passed by reference.

5) QUESTION: Why does the printArea method in `RectangleHelper` not print the correct area?
    - The 'rectangleHelper' is a different instance inheriting the Rectangle class. This instance inherits the default height, width, centre x and centre y params of 0.  

Part B
------

A package `topics_masterclass` is provided in the [tutorials/week10/starter directory](../../tutorials/week10/starter),link it to your workspace.
To copy:
```bash
cd ~/catkin_ws/src/
ln -s <path-to-git>/tutorials/week10/starter/topics_masterclass
```
Build the package using the `catkin_make` command (what folder do you need to be in to execute this command?).

You will also need the stage_ros package, install it by below (replace kinetic with melocdic if you have 18.04 and ROS melodic)
```bash
sudo apt-get install ros-kinetic-stage-ros
```
Start roscore in one terminal
```bash
roscore
```
Start stage_ros in another terminal (replace kinetic with melocdic if you have 18.04 and ROS melodic)
```bash
rosrun stage_ros stageros /opt/ros/kinetic/share/stage/worlds/simple.world
```
You should have the simulator appear
![Simple World in Stage Simulator](http://4.bp.blogspot.com/_B6REL4AVpFA/Szk9ipweWTI/AAAAAAAAALc/orflaXzpcZk/s400/Picture+2.png)

The goal is to modify the provided package to obtain pose of robot (x,y yaw) from nav_msgs/Odometry.

1) QUESTION: How would you access orientation in the `nav_msgs/Odometry` message (HINT: rosmsg show). The answer needs full path to orinettaion from the msg (for String we had msg.data)
    - geometry_msgs/Quaternion 
    - pose.pose.orientation
2) QUESTION: What type of message is the orientation?
    - Quaternion

3) QUESTION: Where is time of this message stored?
    - Time stamp inside std_msgs/Header header

4) TASK: Use the [ROS TF] library helper function to get yaw from the orientation and print to screen (standard out)

5) TASK: Print the pose of robot (x,y yaw) to screen using ROS_INFO_STREAM

[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html

