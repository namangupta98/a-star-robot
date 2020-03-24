# A-Star Robot
[![Build Status](https://travis-ci.org/namangupta98/a-star-robot.svg?branch=master)](https://travis-ci.org/namangupta98/a-star-robot)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/namangupta98/a-star-robot/blob/master/LICENSE)

## Authors
- [Umang Rastogi](https://github.com/urastogi885/)
- [Naman Gupta](https://github.com/namangupta98/)

## Overview
This project is an implementation of A* algorithm on a rigid non-holonomic robot to traverse through a known environment. The project is divided into three main phases.

- Phase-1: Installation of ROS and V-REP
- Phase-2: Implementation of A* on a non-holonomic robot


<p align="center">
  <img src="https://github.com/namangupta98/a-star-robot/blob/master/images/exploration.gif">
  <br><b>Figure 1 - Exploration of a non-holonomic rigid robot using A* to find optimal path from start to goal</b><br>
</p>

## For TAs

- Further improvements have been made to the project after the submission for Project-3 Phase-2.
- The exploration has been reduced to 10 seconds when using dijkstra from 2 hours for the start and goal node
configuration: 50,30,60 and 150,150,0 respectively.
- You can access the submission version from the [*release section*](https://github.com/namangupta98/a-star-robot/releases)

## Dependencies

- Python3
- Python3-tk
- Python3 Libraries: Numpy, OpenCV-Python, Math

## Install Dependencies

- Install Python3, Python3-tk, and the necessary libraries: (if not already installed)

```
sudo apt install python3 python3-tk
sudo apt install python3-pip
pip3 install numpy opencv-python
```

- Check if your system successfully installed all the dependencies
- Open terminal using Ctrl+Alt+T and enter python3.
- The terminal should now present a new area represented by >>> to enter python commands
- Now use the following commands to check libraries: (Exit python window using Ctrl+Z if an error pops up while running the below commands)

```
import tkinter
import numpy
import cv2
```

## Run Instructions

- Using the terminal, clone this repository and go into the project directory, and run the main program:

```
git clone https://github.com/namangupta98/a-star-robot
cd a-star-robot
```

- If you have a compressed version of the project, extract it, go into project directory, open the terminal, and type:

```
python3 a_star_robot.py start_x,start_y,start_orientation goal_x,goal_y,goal_orientaion robot_radius,clearance step_size theta animation
python3 a_star_robot.py 50,30,60 150,150,0 1,1 1 30 1
```

<p align="center">
  <img src="https://github.com/namangupta98/a-star-robot/blob/master/images/final_path.png">
  <br><b>Figure 1 - Final path using A* from start to goal</b><br>
</p>

- Note that the project does not consider the orientation of the robot at the goal point. The goal orientation parameter
will be considered in the next phase of the project.
