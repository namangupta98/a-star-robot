from math import sqrt


class Explorer:
    def __init__(self, start_node, goal_node):
        """
        Initialize the explorer with a start node and final goal node
        :param start_node: a tuple of start coordinates and orientation provided by the user
        :param goal_node: a tuple of goal coordinates and orientation provided by the user
        """
        # Store puzzle and goal nodes as class# Check whether the node is within the square members
        self.start_node = start_node
        self.goal_node = goal_node
        # Define empty lists to store open and closed nodes
        self.open_nodes = []
        self.closed_nodes = []
        self.generated_nodes = []

    def get_heuristic_score(self, node):
        """
        Implement heuristic function for a-star by calculating euclidean distance
        Heuristic is nothing but cost to goal
        :param: node: tuple containing coordinates of the current node
        :return: distance between the goal node and current node
        """
        # Evaluate euclidean distance between goal node and current node and return it
        return sqrt((self.goal_node[0] - node[0])**2 + (self.goal_node[1] - node[1])**2)

    def get_final_weight(self, node, node_cost):
        """
        Get final weight for a-star
        :param node: tuple containing coordinates of the current node
        :param node_cost: cost of each node
        :return: final weight for according to method
        """
        # Add cost-to-goal and cost-to-come to get final cost and return it
        return self.get_heuristic_score(node) + node_cost

    def generate_path(self):
        """
        Generate path using backtracking
        :return: a list of all path nodes
        """
        # Define empty list to store path nodes
        # This list will be used to generate the node-path text file
        path_list = []
        # Get all data for goal node
        last_node = self.closed_nodes[-1]
        # Append the matrix for goal node
        path_list.append(last_node.data)
        # Iterate until we reach the initial node
        while not last_node.data == self.start_node:
            # Search for parent node in the list of closed nodes
            for node in self.closed_nodes:
                if node.data == last_node.parent:
                    # Append parent node
                    # print('Weight:', last_node.weight, last_node.level)
                    path_list.append(last_node.parent)
                    # Update node to search for next parent
                    last_node = node
                    break
        # Return list containing all path nodes
        return path_list
