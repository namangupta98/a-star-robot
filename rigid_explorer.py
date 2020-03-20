# Import standard libraries
import ast
from sys import argv
from time import time
from bisect import insort_left
from cv2 import imshow, waitKey, circle, arrowedLine, line
# Import custom-built classes
from utils.constants import scaling_factor, goal_thresh
from utils.obstacle_space import Map
from utils.explorer import Explorer
from utils.node import Node

script, start_node_coords, goal_node_coords, robot_params, step_size, theta = argv

if __name__ == '__main__':
    start_time = time()
    # Convert arguments into their required data types
    start_node_coords = tuple(ast.literal_eval(start_node_coords))
    start_node_coords = scaling_factor * start_node_coords[0], scaling_factor * start_node_coords[1]
    goal_node_coords = tuple(ast.literal_eval(goal_node_coords))
    goal_node_coords = scaling_factor * goal_node_coords[0], scaling_factor * goal_node_coords[1]
    robot_params = tuple(ast.literal_eval(robot_params))
    obstacle_map = Map(int(robot_params[0]), int(robot_params[1]))
    # Initialize the explorer class
    explorer = Explorer(start_node_coords, goal_node_coords)
    # Check validity of start and goal nodes
    if not (obstacle_map.check_node_validity(start_node_coords[0], obstacle_map.height - start_node_coords[1])
            and obstacle_map.check_node_validity(goal_node_coords[0], obstacle_map.height - goal_node_coords[1])):
        print('One of the points lie in obstacle space!!\nPlease try again')
        quit()
    # Get the start node and add it to open nodes
    start_node = Node(start_node_coords, explorer.get_final_weight(start_node_coords, 0), 0, None)
    explorer.open_nodes.append(start_node)
    explorer.generated_nodes.append(start_node)
    while len(explorer.open_nodes):
        current_node = explorer.open_nodes.pop(0)
        explorer.closed_nodes.append(current_node)
        # Check if current node is the goal node
        if explorer.get_heuristic_score(current_node.data) <= goal_thresh:
            break
        # Generate child nodes and iterate through them
        for child_node in current_node.generate_child_nodes(int(step_size), int(theta),
                                                            (obstacle_map.width, obstacle_map.height)):
            node_repeated = False
            if obstacle_map.check_node_validity(child_node.data[0], obstacle_map.height - child_node.data[1]):
                # Update final weight of the child node
                child_node.weight = explorer.get_final_weight(child_node.data, child_node.cost)
                # Check for repetition of child node in closed nodes
                for closed_node in explorer.closed_nodes:
                    if closed_node.data == child_node.data:
                        node_repeated = True
                        break
                # Check for repetition of child node in open nodes
                for i in range(len(explorer.open_nodes)):
                    if explorer.open_nodes[i].data == child_node.data:
                        if explorer.open_nodes[i].weight > child_node.weight:
                            explorer.open_nodes[i] = child_node
                        node_repeated = True
                        break
                # Append child node to the list of open nodes
                # Do no append child node if repeated
                if not node_repeated:
                    # Maintain a sorted array
                    insort_left(explorer.open_nodes, child_node)
                    # Append latest child node to list that contains all generated nodes
                    explorer.generated_nodes.append(child_node)

    # Generate path
    path_data = explorer.generate_path()
    print('Exploration Time:', time() - start_time)
    start_time = time()
    # Get map to show various obstacles
    map_img = obstacle_map.get_map()
    blue = [255, 0, 0]
    red = [0, 0, 255]
    skip_one = 0
    # Show all generated nodes
    for node in explorer.generated_nodes:
        if not skip_one:
            skip_one += 1
            continue
        arrowedLine(map_img, (node.parent[0], obstacle_map.height - node.parent[1]),
                    (node.data[0], obstacle_map.height - node.data[1]), blue)
        imshow("Node Exploration", map_img)
        waitKey(1)
    # Draw goal node on the map
    circle(map_img, (goal_node_coords[0], obstacle_map.height - goal_node_coords[1]), goal_thresh, [0, 255, 0], -1)
    # Show generated path
    for i in range(1, len(path_data)):
        line(map_img, (path_data[i - 1][0], obstacle_map.height - path_data[i - 1][1]),
             (path_data[i][0], obstacle_map.height - path_data[i][1]), red)
        circle(map_img, (path_data[i][0], obstacle_map.height - path_data[i][1]), goal_thresh, red, -1)
        imshow("Node Exploration", map_img)
    print('Animation Time:', time() - start_time)
    waitKey(15000)
