# Import necessary standard libraries
from math import sqrt
from numpy import sin, cos, pi
# Import necessary constants
from utils.constants import scaling_factor, max_actions, half_actions


def get_child(action_coords, map_limits):
    """
    Get coordinates of the child node
    :param action_coords: a tuple of child node coordinates due to action on parent
    :param map_limits: fa tuple of largest coordinates in the map
    :return: a tuple containing coordinates of the child node
    """
    # Check if new position of 0 are within the array and return coordinates of the child node
    if 0 <= action_coords[0] < map_limits[0] and 0 <= action_coords[1] < map_limits[1]:
        return action_coords

    return None


class Node:
    def __init__(self, node_coordinates, node_weight, node_cost, parent_node):
        """
        Initialize node class with start node and weight of the start node
        :param node_coordinates: a tuple containing x,y coordinates and theta orientation of the node
        :param node_weight: final weight of the node
        :param node_cost: cost of the node
        :param parent_node: store coordinates and orientation of the parent node
        """
        # Define parameters that are common across the class
        self.data = node_coordinates
        self.weight = node_weight
        self.cost = node_cost
        self.parent = parent_node

    def __eq__(self, other):
        return self.weight == other.weight

    def __gt__(self, other):
        return not (self.weight > other.weight)

    def __lt__(self, other):
        return not (self.weight < other.weight)

    def generate_child_nodes(self, step_size, theta, map_limits):
        """
        Generate child nodes of the current node
        :param step_size: No. of units by which the robot should move
        :param theta: angular step between each action
        :param map_limits: a tuple of largest coordinates in the map
        :return: a list of all child nodes
        """
        # Define an empty dictionary to store child nodes
        child_nodes = []
        # Perform each action on the current node to generate child node
        for i in range(max_actions):
            child_data = get_child(self.take_action(i, step_size, theta), map_limits)
            # Check if child node is generated
            if child_data is not None:
                # Define all the properties of the child node and append to the child nodes' list
                child_node = Node(child_data, float('inf'), self.get_child_base_cost(child_data), self.data)
                child_nodes.append(child_node)

        return child_nodes

    def get_child_base_cost(self, child_coords):
        """
        Get base cost of child node
        :param child_coords: a tuple of parent node coordinates
        :return: parent cost + euclidean distance between parent node and child node
        """
        # Self-data contains coordinates of the parent node as a tuple
        return self.cost + sqrt((child_coords[0] - self.data[0]) ** 2 + (child_coords[1] - self.data[1]) ** 2)

    def take_action(self, action, step_size, step_theta):
        """
        Call various actions based on an integer
        :param action: Varies from 0-n to call one of the 8 defined actions
        :param step_theta: angular step between each action
        :param step_size: No. of units by which the robot should move
        :return: new coordinates of the node after the desired action
        """
        # Convert unit of angle from degrees to radians
        if action <= half_actions:
            theta = self.data[2] + (action * step_theta)
            if theta >= 360:
                theta = theta - 360
        else:
            theta = self.data[2] - ((action - half_actions) * step_theta)
            if theta < 0:
                theta = 360 + theta
        # Return the coordinates and orientation of the child node
        return (self.data[0] + int(scaling_factor * step_size * cos(pi * theta / 180)),
                self.data[1] + int(scaling_factor * step_size * sin(pi * theta / 180)),
                theta)
