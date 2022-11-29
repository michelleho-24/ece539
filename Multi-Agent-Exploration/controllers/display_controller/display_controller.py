"""display_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot
from controller import Supervisor
from controller import Display

GREEN = 0x22BB11
WHITE = 0x000000
GROUND_X = 1
GROUND_Z = 1

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
display = robot.getDevice("ground_display")

arena_node = robot.getFromDef("ARENA")
boundary_center = arena_node.getField("translation").getSFVec3f()
boundary_dim = arena_node.getField("floorSize").getSFVec2f()

print("Bottom Left Boundary: ", boundary_center[0] - boundary_dim[0]/2)
BR = [boundary_center[0] + boundary_dim[0]/2, boundary_center[1] - boundary_dim[1]/2]
TR = [boundary_center[0] + boundary_dim[0]/2, boundary_center[1] + boundary_dim[0]/2]
agent_1 = robot.getFromDef("agent_1")
tracker = agent_1.getField("translation")

display_width = display.getWidth()
display_height = display.getHeight()
print("Display Width: ", display_width)
print("Display Height: ", display_height)


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    curr_pos = tracker.getSFVec3f()

    plot_pos = [((math.fabs(curr_pos[0]-BR[0]))/boundary_dim[0])*display_width, ((math.fabs(curr_pos[2]-BR[0]))/boundary_dim[0])*display_height]
    # print(plot_pos)
    # print(int(plot_pos[0]), int(plot_pos[1]))
    display.setColor(GREEN)
    display.fillOval(int(plot_pos[0]), int(plot_pos[1])-40,1,1)
    

# Enter here exit cleanup code.
