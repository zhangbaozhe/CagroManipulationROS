# ECE4310 Project 2: Cargo Manipulation at a Warehouse

This is the repository containing the ROS workspace for the second project of ECE4310 Programming for Robotics at CUHK(SZ). There are two branches:
- master (containing the configuration of Gazebo simulation)
- real_arm (containing the configuration of real robot arm)



## Environment Setup
- Ubuntu 20.04
- ROS noetic
- ...

## Run the code
First, no matter which branch you are at, you need to first compile the workspace. 
```bash
$ catkin_make
```

The running commands are all compressed into one single shell script. For Gazebo simulation you may call:
```bash
$ ./run_gazebo.sh
```
For the real robot arm, you may call:
```bash
$ ./run_real.sh
```

Then, several XTerm windows will be created. 


## Project Description
The task for this project is to pick and place the cubes on the table to the storing area. The cubes have three colors: red, green, and blue. The storing area contains three bins with the three corresponding colors. The cube should be picked and placed to the bin with the same color.

The robot arm is equipped with a camera which can produce 640x480 images. With this camera, the robot arm can use computer vision techniques to first detect the cubes and manage to pick them up and place them into bins. The node for color detection of the cubes is given. 



## Solution
**Detection of the yellow picking area** 
Given the node of detecting the colors of the cubes, we may add another color, yellow, for detecting the picking area. The correct detection of the yellow picking area can help the camera on the robot arm obtain the relative positions of the cubes on the picking area, and further calculate the coordinates at the world frame. 

**Working procedure**
There are two main tasks for the robot arm: color detection and sorting (picking and placing). First, the robot will enter `Hold` phase if there is no cube to be sorted. In `Hold` phase, the robot arm will hold with a specific attitude which allows the camera to capture the images of the full picking area. Color detection will occur at this phase, in which the sorting stacks containing the target positions for the arm will be filled if there are available target cubes. Then, if the cubes are detected, the arm will enter `Sort` phase, in which the detected cubes will be picked up and placed into the bins with the order of the colors red, green, and blue. 