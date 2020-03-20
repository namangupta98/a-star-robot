from numpy import sin, cos, pi
from utils.constants import scaling_factor


def take_action(action, coordinates, step_size, step_theta):
    """
    Call various actions based on an integer
    :param action: Varies from 0-n to call one of the 8 defined actions
    :param coordinates: a tuple containing x-y coordinates of the node
    :param step_theta: angular step between each action
    :param step_size: No. of units by which the robot should move
    :return: new coordinates of the node after the desired action
    """
    # Convert unit of angle from degrees to radians
    theta = action * pi * (step_theta / 180)
    return (coordinates[0] + int(scaling_factor * step_size * cos(theta)),
            coordinates[1] + int(scaling_factor * step_size * sin(theta)))
