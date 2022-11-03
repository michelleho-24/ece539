"""single_agent controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
from controller import Compass
from controller import Gyro

import sys
import platform
import math
import numpy as np
import igraph as ig
import pkg_resources

from rrt_limited import rrt_limited
from movement import Movement
import graph_builder

num_robots = 1
num_obs = 8
obs_size = 0.1 + 0.3 #size of obstacle in meters + footprint
robot_footprint = 0.05 #Robot size is 0.03 x 0.02 meters, approximate as 0.05 m square
sample_space = robot_footprint*2 #Space to be sample from the robot, radius of a circle in meters
sample_counter= 0 #Used to track the edge IDs
# boundary_center = [0.0201, -0.417] # Size of arena is 1 x 1.4 (x,y) meters
# arena_size = [1, 1.4]
curr_vertex="home" #Tracks the name of the current vertex 

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motorL = robot.getDevice("left motor")
motorR = robot.getDevice("right motor")

compass = Compass("compass")
gyro = Gyro("gyro")

motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))

defVal="agent_1"
obs_node_array = np.empty(num_obs, dtype=object)
obs_pos_array = np.empty(num_obs, dtype=object)
g = ig.Graph()

agent_node = robot.getFromDef(defVal)
trans_field = agent_node.getField("translation")
trans_value = trans_field.getSFVec3f()
compass.enable(8)
gyro.enable(8)

for i in range(num_obs):
    obs_node_array[i] = robot.getFromDef("obs" + str(i))
    obs_pos_array[i] = obs_node_array[i].getField("translation").getSFVec3f()
    obs_pos_array[i].append(obs_size)
    #print(obs_pos_array[i])

arena_node = robot.getFromDef("arena")
boundary_center = arena_node.getField("translation").getSFVec3f()
boundary_dim = arena_node.getField("floorSize").getSFVec2f()
boundary_dim[0] = boundary_dim[0] - 0.05 
boundary_dim[1] = boundary_dim[1] - 0.05 

rrt_planner = rrt_limited(obs_pos_array, sample_space, boundary_center, boundary_dim)
mvController = Movement(robot, motorL, motorR, compass, gyro)

g.add_vertex(name="home", pos=trans_field.getSFVec3f()) #Add the home position, starting point

#Data logging variables
robot_pos_log = []

print("Version: ", sys.version)
curr_pos = trans_field.getSFVec3f()
sampled_point = rrt_planner.expand_rrt(curr_pos)
mvController.moveToDestination(sampled_point, curr_pos)

while robot.step(timestep) != -1:

    # #Update velocity after every simulation step
    
    # robot_pos_log.append(curr_pos)

   

    # #Perform outbound expansion
    # g, curr_vertex, sample_counter = graph_builder.outbound_expansion(g, curr_vertex, sampled_point, sample_counter)

    # #Move to newly sampled destination
    

    # #Maintian safe recursive feasibility
    # #graph_builder.inbound_consolidation(curr_pos, sample_space) 
    pass
    

# Enter here exit cleanup code.
with open('obs_pos.txt', 'w') as f:
    for line in obs_pos_array:
        f.write(f"{line}\n")
f.close()

with open('robot_pos.txt', 'w') as f:
    for line in robot_pos_log:
        f.write(f"{line}\n")
f.close()

ig.plot(g, target='myfile.pdf')

        
