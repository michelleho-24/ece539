from rrt_limited import rrt_limited

obs_pos_array = [[-0.117,0.05,-0.0971,0.15],[-0.102, 0.05, -0.344,0.15],[-0.0962, 0.05, -0.432,0.15],[-0.029, 0.05, -0.433,0.15],[0.147, 0.05, -0.324,0.15],[0.136, 0.05, -0.216,0.15],[0.122, 0.05, -0.104,0.15],[-0.104, 0.05, -0.223,0.15]]
robot_footprint = 0.25 #Robot size is 0.03 x 0.02 meters, approximate as 0.05 m square
sample_space = robot_footprint*1.25 #Space to be sample from the robot, radius of a circle in meters
boundary_center = [0.0201, 0.0, -0.417]
boundary_dim = [0.9, 1.3]

rrt_planner = rrt_limited(obs_pos_array, sample_space, boundary_center, boundary_dim)
curr_pos = [-0.235,0.0104,0.031]
goal_pos = [0.161, 0.0104,-0.769]

rrt_planner.rrt_iterate(curr_pos)


#obstacles
# [-0.117,0.05,-0.0971] [-0.102, 0.05, -0.344] [-0.0962, 0.05, -0.432] [-0.029, 0.05, -0.433] [0.147, 0.05, -0.324] [0.136, 0.05, -0.216] [0.122, 0.05, -0.104] [-0.104, 0.05, -0.223]
# radius of 0.15