"""single_agent controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
from controller import Compass
from controller import Gyro
from controller import InertialUnit

import sys
import platform
import math
import numpy as np
import igraph as ig
import pkg_resources
import random
import csv

from rrt_limited import rrt_limited
from movement import Movement
import graph_builder

num_robots = 1
num_obs = 8
obs_size = 0.15 #size of obstacle in meters + footprint
robot_footprint = 0.25 #Robot size is 0.03 x 0.02 meters, approximate as 0.05 m square
sample_space = robot_footprint*1.25 #Space to be sample from the robot, radius of a circle in meters
sample_counter= 1 #Used to track the edge IDs
eps_inbound = 0.4
MAX_TIME = 60 #900seconds = 15min
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
imu = InertialUnit("inertial unit")

motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))

defVal="agent_1"
obs_node_array = np.empty(num_obs, dtype=object)
obs_pos_array = np.empty(num_obs, dtype=object)
g_f = ig.Graph(directed=True)
g_b = ig.Graph(directed=True)

agent_node = robot.getFromDef(defVal)
trans_field = agent_node.getField("translation")
trans_value = trans_field.getSFVec3f()
compass.enable(1)
gyro.enable(8)
imu.enable(1)

for i in range(num_obs):
    obs_node_array[i] = robot.getFromDef("obs" + str(i))
    obs_pos_array[i] = obs_node_array[i].getField("translation").getSFVec3f()
    obs_pos_array[i].append(obs_size)
    #print(obs_pos_array[i])

arena_node = robot.getFromDef("arena")
boundary_center = arena_node.getField("translation").getSFVec3f()
boundary_dim = arena_node.getField("floorSize").getSFVec2f()
boundary_dim[0] = boundary_dim[0] - 0.1
boundary_dim[1] = boundary_dim[1] - 0.1

print("Boundary Center: ", boundary_center)
print("Boundary Dimensions: ", boundary_dim)

rrt_planner = rrt_limited(obs_pos_array, sample_space, boundary_center, boundary_dim)
mvController = Movement(robot, motorL, motorR, compass, gyro, imu)

g_f.degree(mode="in")
g_f.add_vertex(name="home", pos=trans_field.getSFVec3f()) #Add the home position, starting point
print(g_f)
#Data logging variables
robot_pos_log = []
rrt_sample_log = []

# with open('obs_pos.txt', 'w') as f:
#     for line in obs_pos_array:
#         f.write(f"{line}\n")
# f.close()

print("Version: ", sys.version)
# curr_pos = trans_field.getSFVec3f()
# sampled_point = rrt_planner.expand_rrt(curr_pos)
# mvController.moveToDestination(sampled_point, curr_pos

#mvController.motorRotateLeft()
#mvController.motorMoveForward()
sim_time = robot.getTime()
# dest = [-0.00467573, 0.011, 0.15998]
dest = [-0.23, 0.011, -0.219]
mvController.moveToDestination(dest, trans_field.getSFVec3f())
# mvController.motorMoveForward()
while robot.step(timestep) != -1:
    pass
    # print(agent_node.getVelocity())
    # curr_pos = trans_field.getSFVec3f()
    # sampled_point = rrt_planner.expand_rrt(curr_pos)

    # rrt_sample_log.append(sampled_point)
    # robot_pos_log.append(curr_pos)

    # # #Perform outbound expansion
    # g_f, curr_vertex, sample_counter = graph_builder.outbound_expansion(g_f, curr_vertex, sampled_point, sample_counter)

    # # #Move to newly sampled destination
    # mvController.moveToDestination(sampled_point, curr_pos)
    # eps = random.uniform(0,1)
    # # #Maintian safe recursive feasibility
    # if eps < eps_inbound:
    #     print("Inbound Consolidation")
    #     g_f, g_b, curr_vertex = graph_builder.inbound_consolidation(curr_pos, g_f, g_b, curr_vertex, mvController) 
    
    # if (robot.getTime() - sim_time > MAX_TIME):
    #     # Save experiment data
    #     with open('robot_pos.txt', 'w') as f:
    #         # creating a csv writer object
    #         csvwriter = csv.writer(f)

    #         # writing the data rows``
    #         csvwriter.writerows(robot_pos_log)
    #     f.close()

    #     with open('rrt_sample.txt', 'w') as f:
    #         # creating a csv writer object
    #         csvwriter = csv.writer(f)

    #         # writing the data rows``
    #         csvwriter.writerows(rrt_sample_log)
    #     f.close()

    #     #ig.plot(g_f, "forward.png")

    #     robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)



        
