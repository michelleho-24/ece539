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
# import igraph as ig
from movement import Movement

num_robots = 1

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
# motorL.setPosition(0.1)
defVal="agent_1"
rov_node_array = np.empty(num_robots, dtype=object)
trans_field_array = np.empty(num_robots, dtype=object)
trans_value_array = np.empty(num_robots, dtype=object)

agent_node = robot.getFromDef(defVal)
trans_field = agent_node.getField("translation")
trans_value = trans_field.getSFVec3f()
compass.enable(8)
gyro.enable(8)
mvController = Movement(robot, motorL, motorR, compass, gyro)

# for i in range(num_robots):
#     trans_field_array[i] = rov_node_array[i].getField("translation")
#     trans_value_array[i] = trans_field_array[i].getSFVec3f()
#     compass.enable(8)
    

destination=[-0.0256, -0.112]
print(sys.version)
print(platform.python_version())
while robot.step(timestep) != -1:
    #Update velocity after every simulation step
    curr_pos = trans_field.getSFVec3f()
    #print(agent_node.getVelocity())
    # motorRotateRight()
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    mvController.motorMoveForward()
    #mvController.moveToDestination(destination, curr_pos)
    

# Enter here exit cleanup code.
