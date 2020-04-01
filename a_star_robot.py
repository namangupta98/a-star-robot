# Import necessary standard libraries
import ast
from sys import argv
from time import time
# Import necessary custom-built classes and methods
from utils.obstacle_space import Map
from utils.constants import scaling_factor
from utils.explorer import Explorer, check_node_validity

"""
Add various parameters as input arguments from user
:param start_node_data: a tuple of 3 values: start coordinates and orientation
:param goal_node_data: a tuple of 3 values: goal coordinates and orientation
:param robot_params: a tuple of 2 values: robot radius and clearance
:param step_size: integer value from 1-10 for translation of the robot
:param theta: angular step between each action
:param animation: 1 to show animation otherwise use 0
"""
script, start_node_data, goal_node_data, robot_params, step_size, theta, animation = argv

if __name__ == '__main__':
    # Convert input arguments into their required data types
    start_node_data = tuple(ast.literal_eval(start_node_data))
    start_node_data = (scaling_factor * start_node_data[1], scaling_factor * start_node_data[0],
                       start_node_data[2] // int(theta))
    goal_node_data = tuple(ast.literal_eval(goal_node_data))
    goal_node_data = (scaling_factor * goal_node_data[1], scaling_factor * goal_node_data[0],
                      goal_node_data[2] // int(theta))
    robot_params = tuple(ast.literal_eval(robot_params))
    # Initialize the map class
    obstacle_map = Map(int(robot_params[0]), int(robot_params[1]))
    check_image = obstacle_map.check_img
    # Initialize the explorer class
    explorer = Explorer(start_node_data, goal_node_data, int(step_size), int(theta))
    # Check validity of start and goal nodes
    if not (check_node_validity(check_image, start_node_data[1], obstacle_map.height - start_node_data[0])
            and check_node_validity(check_image, goal_node_data[1], obstacle_map.height - goal_node_data[0])):
        print('One of the points lie in obstacle space!!\nPlease try again')
        quit()
    # Get start time for exploration
    start_time = time()
    # Start exploration
    explorer.explore(check_image)
    # Show time for exploration
    print('Exploration Time:', time() - start_time)
    if int(animation) == 1:
        # Get start time for animation
        start_time = time()
        # Display animation of map exploration to find goal
        explorer.show_exploration(obstacle_map.obstacle_img)
        # Show time for animation
        print('Animation Time:', time() - start_time)
