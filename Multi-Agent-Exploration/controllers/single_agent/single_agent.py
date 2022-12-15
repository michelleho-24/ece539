"""single_agent controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
from controller import Compass
from controller import Gyro
from controller import InertialUnit

from controller import Display

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

curr_vertex="home" #Tracks the name of the current vertex 

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motorL = robot.getDevice("left motor")
motorR = robot.getDevice("right motor")

lDist = robot.getDevice("left distance sensor")
rDist = robot.getDevice("right distance sensor")

lDist.enable(timestep)
rDist.enable(timestep)

compass = Compass("compass")
gyro = Gyro("gyro")
imu = InertialUnit("inertial unit")

motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))

defVal="agent_" + sys.argv[1]
print(defVal)

obs_node_array = np.empty(num_obs, dtype=object)
obs_pos_array = np.empty(num_obs, dtype=object)
g_f = ig.Graph(directed=True)
g_b = ig.Graph(directed=True)

agent_node = robot.getFromDef(defVal)
trans_field = agent_node.getField("translation")
trans_value = trans_field.getSFVec3f()
compass.enable(1)
gyro.enable(1)
imu.enable(1)

for i in range(num_obs):
    obs_node_array[i] = robot.getFromDef("obs" + str(i))
    obs_pos_array[i] = obs_node_array[i].getField("translation").getSFVec3f()
    obs_pos_array[i].append(obs_size)


arena_node = robot.getFromDef("ARENA")
boundary_center = arena_node.getField("translation").getSFVec3f()
boundary_dim = arena_node.getField("floorSize").getSFVec2f()
boundary_dim[0] = boundary_dim[0] - 0.3
boundary_dim[1] = boundary_dim[1] - 0.3

print("Boundary Center: ", boundary_center)
print("Boundary Dimensions: ", boundary_dim)

rrt_planner = rrt_limited(obs_pos_array, sample_space, boundary_center, boundary_dim)
mvController = Movement(robot, agent_node, motorL, motorR, compass, gyro, imu, lDist, rDist)

g_f.degree(mode="in")
g_f.add_vertex(name="home", pos=trans_field.getSFVec3f()) #Add the home position, starting point

#Data logging variables
robot_pos_log = []
rrt_sample_log = []

print("Version: ", sys.version)

sim_time = robot.getTime()

while robot.step(timestep) != -1:
    
    curr_pos = trans_field.getSFVec3f()
    sampled_point = rrt_planner.expand_rrt(curr_pos)

    rrt_sample_log.append(sampled_point)
    print("Sample point: ", sampled_point)
    robot_pos_log.append(curr_pos)

    # #Perform outb ound expansion
    g_f, curr_vertex, sample_counter = graph_builder.outbound_expansion(g_f, curr_vertex, sampled_point, sample_counter)

    # #Move to newly sampled destination
    mvController.moveToDestination(sampled_point, curr_pos)
    print("Destination: ", trans_field.getSFVec3f())
    eps = random.uniform(0,1)
    # #Maintain safe recursive feasibility
    if eps < eps_inbound:
        print("Inbound Consolidation")
        g_f, g_b, curr_vertex = graph_builder.inbound_consolidation(g_f, g_b, curr_vertex, mvController, trans_field) 
    
    if (robot.getTime() - sim_time > MAX_TIME):
        # Save experiment data
        with open('robot_pos.txt', 'w') as f:
            # creating a csv writer object
            csvwriter = csv.writer(f)

            # writing the data rows``
            csvwriter.writerows(robot_pos_log)
        f.close()

        with open('rrt_sample.txt', 'w') as f:
            # creating a csv writer object
            csvwriter = csv.writer(f)

            # writing the data rows``
            csvwriter.writerows(rrt_sample_log)
        f.close()

        #ig.plot(g_f, "forward.png")

        robot.simulationSetMode(robot.SIMULATION_MODE_PAUSE)



        
