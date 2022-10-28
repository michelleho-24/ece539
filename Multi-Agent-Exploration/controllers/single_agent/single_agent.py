"""single_agent controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
from controller import Compass
import math
import numpy as np

num_robots = 1
COORDINATE_MATCHING_ACCURACY = 0.01
THETA_MATCHING_ACCURACY = 1
ROBOT_ANGULAR_SPEED_IN_DEGREES = 18
TANGENSIAL_SPEED = 0.128
MAX_SPEED = 10


# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motorL = robot.getDevice("left motor")
motorR = robot.getDevice("right motor")

compass = Compass("compass")

motorL.setPosition(float('+inf'))
motorR.setPosition(float('+inf'))
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
motorL.setPosition(0.1)
defArray=["agent_1"]
rov_node_array = np.empty(num_robots, dtype=object)
trans_field_array = np.empty(num_robots, dtype=object)
trans_value_array = np.empty(num_robots, dtype=object)

def motorStop():
    motorL.setVelocity(0)
    motorR.setVelocity(0)

def motorMoveForward():
    motorL.setVelocity(MAX_SPEED)
    motorR.setVelocity(MAX_SPEED)

def motorRotateLeft():
    motorL.setVelocity(-MAX_SPEED)
    motorR.setVelocity(MAX_SPEED)

def motorRotateRight():
    motorL.setVelocity(MAX_SPEED)
    motorR.setVelocity(-MAX_SPEED)

def moveForward(distance):
	# the duration required for the robot to move by the specified distance
	duration = distance / TANGENSIAL_SPEED
	print("duration to reach target location: %.5f\n", duration)

	#set robot motor to move forward
	motorMoveForward()

	#run the simulator
	start_time = robot.getTime()
	while True:
           if (robot.getTime() >= start_time + duration):
               break
           robot.step(8)
	#stop the motor
	motorStop()
	robot.step(8)

def rotateHeading(thetaDot):
	if not (cartesianIsThetaEqual(thetaDot, 0)):
		duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES
		print("duration to face the destination: %.5f\n", duration)

		#if thetaDot > 0, robot will rotate to left
		if (thetaDot > 0):
			# set robot motor to rotate left
			motorRotateLeft()
		#if thetaDot < 0, robot will rotate to right
		elif (thetaDot < 0):
			# set robot motor to rotate right
			motorRotateRight()

		# run the simulator
		start_time = robot.getTime()
		while True:
                  if (robot.getTime() >= start_time + duration):
                      break
                  robot.step(8)

#motorR.setPosition(0.1)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
def cartesianConvertCompassBearingToHeading(heading):
    # /* 
	 # * in webots, heading increasement is rotate in clockwise
	 # * in cartesian, heading increasement is rotate in counterclockwise
	 # * so headingInCartesian = 360-headingInWebots
	 # * */
    heading = 360-heading
    
    # /* 
	 # * in webots, heading is 0 if robot faced to y+ axis
	 # * in cartesian, heading is 0 if robot face to x+ axis
	 # * so headingInCartesian = headingInWebots+90
	 # * */
    heading = heading + 90
    if (heading > 360.0):
        heading = heading - 360.0

    return heading


def positioningControllerGetRobotCoordinate():
    position = np.zeros(2)
    position[0] = (trans_field_array[0].getSFVec3f())[0]
    position[1] = -(trans_field_array[0].getSFVec3f())[2]
       
    return position   
    
def cartesianIsCoordinateEqual(coordinate1, coordinate2):
    if (abs(coordinate1[0]-coordinate2[0]) < COORDINATE_MATCHING_ACCURACY and abs(coordinate1[1]-coordinate2[1]) < COORDINATE_MATCHING_ACCURACY):
        return True
    else:
        return False
  
def cartesianIsThetaEqual(theta, theta2):
    if (abs(theta - theta2) < THETA_MATCHING_ACCURACY):
        return True
    else:
        return False
        
def get_bearing_in_degrees():
  north = compass.getValues()
  rad = math.atan2(north[1], north[0])
  bearing = (rad - 1.5708) / math.pi * 180.0
  if (bearing < 0.0):
    bearing = bearing + 360.0
  return bearing


def positioningControllerGetRobotHeading():
    return cartesianConvertCompassBearingToHeading(get_bearing_in_degrees())
    
def cartesianCalcDestinationThetaInDegrees(currentCoordinate,  destinationCoordinate):
    return math.atan2(destinationCoordinate[1] - currentCoordinate[1], destinationCoordinate[0] - currentCoordinate[0]) * 180 / math.pi


def cartesianCalcThetaDot(heading, destinationTheta):
    theta_dot = destinationTheta - heading;

    if (theta_dot > 180):
        theta_dot = -(360-theta_dot)
    elif (theta_dot < -180):
        theta_dot = (360+theta_dot)

    return theta_dot

def cartesianCalcDistance(currentCoordinate, destinationCoordinate):
    return math.sqrt(math.pow(destinationCoordinate[0]-currentCoordinate[0], 2) + math.pow(destinationCoordinate[1]-currentCoordinate[1], 2))


def positioningControllerCalcDistanceToDestination(destinationCoordinate):

	currentCoordinate = positioningControllerGetRobotCoordinate()
	return cartesianCalcDistance(currentCoordinate, destinationCoordinate)

def positioningControllerCalcThetaDotToDestination(destinationCoordinate):

	currentCoordinate = positioningControllerGetRobotCoordinate()
	robotHeading = positioningControllerGetRobotHeading()
	destinationTheta = cartesianCalcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate)
	return cartesianCalcThetaDot(robotHeading, destinationTheta)



def moveToDestination(destinationCoordinate):
    currentCoordinate = positioningControllerGetRobotCoordinate()
    print("Initial Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1])
    print("Destination Coordinate: %.5f %.5f\n", destinationCoordinate[0], destinationCoordinate[1])
	
	# if the robot is already at the destination location
    if (cartesianIsCoordinateEqual(positioningControllerGetRobotCoordinate(), destinationCoordinate)):
        
            printf("Robot is already at the destination location\n")
            return
        
    # thetaDot is the degree of rotation needed by the robot to face the destination
	# thetaDot is zero if robot is facing the destination

    thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate)
    print("thetaDotToDestination: %.5f\n", thetaDotToDestination)

    rotateHeading(thetaDotToDestination)

	# the distance needed for the robot to reach its destination
    distanceToDestination = positioningControllerCalcDistanceToDestination(destinationCoordinate)
    print("distanceToDestination: %.5f\n", distanceToDestination)
    
    moveForward(distanceToDestination)
    
    currentCoordinate = positioningControllerGetRobotCoordinate()
    print("Stop Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1])




for i in range(num_robots):
    rov_node_array[i] = robot.getFromDef(defArray[i])
    trans_field_array[i] = rov_node_array[i].getField("translation")
    trans_value_array[i] = trans_field_array[i].getSFVec3f()
    compass.enable(8)

destination=[0.024, -0.156]
moveToDestination(destination)

while robot.step(timestep) != -1:
    # motorRotateRight()
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
    

# Enter here exit cleanup code.
