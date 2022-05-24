# ECE4310 Project 2: Cargo Manipulation at a Warehouse

This is the repository containing the ROS workspace for the second project of ECE4310 Programming for Robotics at CUHK(SZ).


## Environment Setup
- Ubuntu 20.04
- ROS noetic
- ...

## Project Description
The task for this project is to pick and place the cubes on the table to the storing area. 

### Solution
**Detection of the yellow picking area** Given the node of detecting the colors of the cubes, we may add another color, yellow, for detecting the picking area. The correct detection of the yellow picking area can help the camera on the robot arm obtain the relative positions of the cubes on the picking area, and further calculate the coordinates at the world frame. 