from rrt_limited import rrt_limited
num_robots = 1
num_obs = 8
obs_size = 0.1 + 0.3 #size of obstacle in meters + footprint
robot_footprint = 0.05 #Robot size is 0.03 x 0.02 meters, approximate as 0.05 m square
sample_space =  100#robot_footprint*2 #Space to be sample from the robot, radius of a circle in meters
sample_counter= 1 #Used to track the edge IDs
eps_inbound = 0.4
MAX_TIME = 0.5 #900seconds = 15min
# boundary_center = [0.0201, -0.417] # Size of arena is 1 x 1.4 (x,y) meters
# arena_size = [1, 1.4]
curr_vertex="home" #Tracks the name of

for i in range(100):
    