# include <webots/robot.h>
import igraph as ig

tangential_speed = 0.12874
robot_rotational_spteed = 0.772881647
robot_angular_speed_degrees = 278.237392796

def getTimeStep ():
    time_step = -1
    if (time_step == -1): 
        time_step = int(wb_robot_get_basic_time_step())
    
    return time_step 

def step():
    if(wb_robot_step(time_step) == -1):
        wb_robot_cleanip()
        exit(EXIT_SUCCESS) 

# check indenting on this
def init():
    time_step = getTimeStep()
    motorControllerInit(time_step)
    positioningControllerInit(time_step)

    step()

def rotateHeading (thetaDot):
    # if thetaDot is zero
    if (!cartesianIsThetaEqual(thetaDot, 0)):
        # the duration required for the robot to rotate the body by the specified thetaDot
        duration = abs(thetaDot)/robot_angular_speed_degrees
        printf("duration to face the destination: %.5f\n", duration)

        # if thetaDot > 0, robot will rotate to left
        if (thetaDot > 0):
            # set robot motor to rotate left
            motorRotateLeft()
        elif (thetaDot < 0):
            # set robot motor to rotate right
            motorRotateRight()
        
        # run simulator
        start_time = wb_robot_get_time()
        
        while(wb_robot_get_time() < start_time + duration):
            step()

def moveForward(distance): 
    # duration required for the robot to move by specified distance
    duration = distance/tangential_speed

    # set robot motor to move forward
    motorMoveForward()

    # run simulator
    start_time = wb_robot_get_time()
        
    while(wb_robot_get_time() < start_time + duration):
        step()

def moveToDestination(destinationCoordinate[2]):
    currentCoordinate = positioningControllerGetRobotCoordinate()
    printf("Initial Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1])
    printf("Destination Coordinate: %.5f %.5f\n", destinationCoordinate[0], destinationCoordinate[1])
	
	# if the robot is already at the destination location
    if (cartesianIsCoordinateEqual(positioningControllerGetRobotCoordinate(), destinationCoordinate)):
        
            printf("Robot is already at the destination location\n")
            return
        
    # thetaDot is the degree of rotation needed by the robot to face the destination
	# thetaDot is zero if robot is facing the destination

    thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate)
    printf("thetaDotToDestination: %.5f\n", thetaDotToDestination)

    rotateHeading(thetaDotToDestination)

	# the distance needed for the robot to reach its destination
    distanceToDestination = positioningControllerCalcDistanceToDestination(destinationCoordinate)
    printf("distanceToDestination: %.5f\n", distanceToDestination)
    
    moveForward(distanceToDestination)
    
    currentCoordinate = positioningControllerGetRobotCoordinate()
    printf("Stop Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1])

def main(argc, **argv):
    
    wb_robot_init()
    init()
    
    destinationCoordinate[2] = {0.35, -0.35}
    
    moveToDestination(destinationCoordinate)
    
    wb_robot_cleanup()
    return EXIT_SUCCESS
    






        


