Quiz 10
======

INFORMATION
------
- To develop the quiz 

symbolically link the [quiz10 folder](.) to your catkin_ws/src (ie if your quiz10 folder path is <git_repo_path>/quizzes/quiz10/ then execute:
```bash
cd ~/catkin_ws/src
ln -s <git_repo_path>/quizzes/quiz10/
```
- Compile with catkin_make
```bash
cd ~/catkin_ws
catkin_make
```

- You will need to run the a4_setup to enable testing your code
```bash
roslaunch a4_setup a4_setup.launch
```
- When running your package for Quiz10a
```bash
rosrun random_walk random_walk-sample
```
- When running your package for Quiz10b
```bash
rosrun object_detector object_detector-sample
```
- Sections for TASKS in the quiz have a `(//! @todo)` in code to enable identification

*For part B*

Compile the unit test with
```bash
catkin_make tests
```
Run the test with
```bash
rosrun object_detector object_detector-test 
```
To examine the rosbag that has been supplied, close the simulation, open a terminal and go to the bag folder using `roscd`
```bash
 roscd object_detector/test/bag/
```
Thereafter play the bag using
```bash
rosbag play -r 0.1 --clock -l sample.bag
```
In another terminal And examine the data using (and the config from pfms.rviz) 
```
rosrun rviz rviz
```

Part A
------
1) QUESTION: Look at the code in [random_walk](./a/random_walk), what is the name of the package?

ANS: random_walk

2) QUESTION: Look at the code in [RandomWalk](./a/random_walk/src/sample.cpp) class. The constrcutor subscribes to two topics, what are the two topics ?

ANS: /scan & /cmd_vel

3) TASK: Without changing the code [RandomWalk](./a/random_walk/src/sample.cpp) make the node subscribe to `/robot_0/cmd_vel` and `/robot_0/base_scan`

ANS: rosrun random_walk random_walk-sample scan:=robot_0/base_scan cmd_vel:=robot_0/cmd_vel

4) TASK: In [laserCallback](./a/random_walk/src/sample.cpp) find the closest point range reading on the right and left side of the robot and assign it to `minR` and `minL` respectively.

ANS: Done.

5) QUESTION: Run the `random_walk` node with modifications carried out in step (3) and (4) above. Describe what the code is achieving looking internally at the code? 

ANS: The code compares the detected distance with the maximum detectable range for every angels. If the detected distance is smaller than the maximum detectable range, it is stored as the closet range. And then, that stored value is keep updated according to the reading gained from sensor_msgs/LaserScan. 

Part B
------
1) QUESTION: Look at the code in [folder b](./b/object_detector/src/main.cpp), name the node `object_detector`?

ANS: Done. XXXXXX is changed

2) TASK: In the [DetectCabinet](./b/object_detector/src/detectcabinet.cpp) return true if a high intensity reading is available.  

ANS: Done.

3) TASK: In the [unit test](./b/object_detector/test/utest.cpp) make the test pass if `detectCabinetPresence` returns a true.\

ANS: Done.

4) TASK: In [laserCallback](./a/object_detector/src/sample.cpp) call the `detectCabinet` member function of `DetectCabinet`.

ANS:

5) QUESTION: Playing back the rosbag, and having rviz allows us to look at the data that has been recorded. What shape(s) are the high intensity cabinets?

ANS:
