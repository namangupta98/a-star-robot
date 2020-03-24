# Define minimum distance threshold in map
dist_thresh = 0.5
scaling_factor = int(1 / dist_thresh)
# Define threshold around goal
goal_thresh = int(scaling_factor * 1.5)
# Define map size
width, height = 300, 200
map_size = (scaling_factor * height), (scaling_factor * width)
# Define all the possible no. of actions
max_actions = 5
half_actions = max_actions // 2
# Define total angle of a complete circle
total_angle = 360
# Define exploration constants
no_parent = -1
node_generated = 1
start_parent = -99
