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
#import igraph as ig

from rrt_limited import rrt_limited
from movement import Movement
import graph_builder

num_robots = 1
num_obs = 8
obs_size = 0.1 + 0.3 #size of obstacle in meters + footprint
robot_footprint = 0.05 #Robot size is 0.03 x 0.02 meters, approximate as 0.05 m square
sample_space = 0.25 #Space to be sample from the robot, radius of a circle in meters
sample_period = 5
sample_counter= 0

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

agent_node = robot.getFromDef(defVal)
trans_field = agent_node.getField("translation")
trans_value = trans_field.getSFVec3f()
compass.enable(8)
gyro.enable(8)
mvController = Movement(robot, motorL, motorR, compass, gyro)

#Data logging variables
robot_pos_log = []

for i in range(num_obs):
    obs_node_array[i] = robot.getFromDef("obs" + str(i))
    obs_pos_array[i] = obs_node_array[i].getField("translation").getSFVec3f()
    obs_pos_array[i].append(obs_size)
    print(obs_pos_array[i])

rrt_planner = rrt_limited(obs_pos_array, sample_space)

destination=[0.249, -0.0579]
print("Version: ", sys.version)

while robot.step(timestep) != -1:

    #Update velocity after every simulation step
    curr_pos = trans_field.getSFVec3f()
    print(curr_pos)
    robot_pos_log.append(curr_pos)

    rrt_planner.sample_random(curr_pos)

    # #Perform outbound expansion
    # graph_builder.outbound_expansion(curr_pos, sample_space) 

    # #Move to newly sampled destination
    # mvController.moveToDestination(destination, curr_pos)

    # #Maintian safe recursive feasibility
    # graph_builder.inbound_consolidation(curr_pos, sample_space) 
    

# Enter here exit cleanup code.
with open('obs_pos.txt', 'w') as f:
    for line in obs_pos_array:
        f.write(f"{line}\n")
f.close()

with open('robot_pos.txt', 'w') as f:
    for line in robot_pos_log:
        f.write(f"{line}\n")
f.close()
        
