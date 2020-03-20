# A-Star Robot

## Authors
- [Umang Rastogi](https://github.com/urastogi885/)
- [Naman Gupta](https://github.com/namangupta98/)

## Overview
This project is an implementation of A* algorithm on a rigid non-holonomic robot to traverse through a known environment. The project is divided into three main phases.

- Phase-1: Installation of ROS and V-REP
- Phase-2: Implementation of A* on non-holonomic robot

## Dependencies

- Python3
- Python3-tk
- Python3 Libraries: Numpy, OpenCV-Python, Math

## Install Dependencies

Install Python3, Python3-tk, and the necessary libraries: (if not already installed)

```
sudo apt install python3 python3-tk
sudo apt install python3-pip
pip3 install numpy opencv-python
```
Check if your system successfully installed all the dependencies
    Open terminal using Ctrl+Alt+T and enter python3.
    The terminal should now present a new area represented by >>> to enter python commands
    Now use the following commands to check libraries: (Exit python window using Ctrl+Z if an error pops up while running the below commands)

```
import tkinter
import numpy
import cv2
```

## Run Instructions
Using the terminal, clone this repository and go into the project directory, and run the main program:
```
git clone https://github.com/namangupta98/a-star-robot
cd a-star-robot
```
If you have a compressed version of the project, extract it, go into project directory, open the terminal, and type:
```
python3 rigid_explorer.py start_x,start_y goal_x,goal_y method
python3 rigid_explorer.py 5,5 200,100 a
```

