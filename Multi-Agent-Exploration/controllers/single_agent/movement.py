#LIBRARY IMPORTS
from controller import Robot
from controller import Supervisor
from controller import Compass
import math
import numpy as np


class Movement:
    def __init__(self, robot, motorL, motorR, position):
        self.robot = robot
        self.motorL = motorL
        self.motorR = motorR 
        self.position = position
        #DEFINED CONSTANTS
        COORDINATE_MATCHING_ACCURACY = 0.01
    THETA_MATCHING_ACCURACY = 1
    ROBOT_ANGULAR_SPEED_IN_DEGREES = 18
    TANGENSIAL_SPEED = 0.128
    MAX_SPEED = 10

    def motorStop(self):
        self.motorL.setVelocity(0)
        self.motorR.setVelocity(0)

    def motorMoveForward(self):
        print("moving forward")
        print(self.position)
        self.motorL.setVelocity(self.MAX_SPEED)
        self.motorR.setVelocity(self.MAX_SPEED)

    # def motorRotateLeft():
    #     motorL.setVelocity(-MAX_SPEED)
    #     motorR.setVelocity(MAX_SPEED)

    # def motorRotateRight():
    #     motorL.setVelocity(MAX_SPEED)
    #     motorR.setVelocity(-MAX_SPEED)

    # def moveForward(distance):
    #     # the duration required for the robot to move by the specified distance
    #     duration = distance / TANGENSIAL_SPEED
    #     print("duration to reach target location: %.5f\n", duration)

    #     #set robot motor to move forward
    #     motorMoveForward()

    #     #run the simulator
    #     start_time = robot.getTime()
    #     while True:
    #         if (robot.getTime() >= start_time + duration):
    #             break
    #         robot.step(8)
    #     #stop the motor
    #     motorStop()
    #     robot.step(8)

    # def rotateHeading(thetaDot):
    #     if not (cartesianIsThetaEqual(thetaDot, 0)):
    #         duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES
    #         print("duration to face the destination: %.5f\n", duration)

    #         #if thetaDot > 0, robot will rotate to left
    #         if (thetaDot > 0):
    #             # set robot motor to rotate left
    #             motorRotateLeft()
    #         #if thetaDot < 0, robot will rotate to right
    #         elif (thetaDot < 0):
    #             # set robot motor to rotate right
    #             motorRotateRight()

    #         # run the simulator
    #         start_time = robot.getTime()
    #         while True:
    #                 if (robot.getTime() >= start_time + duration):
    #                     break
    #                 robot.step(8)

    # #motorR.setPosition(0.1)
    # # Main loop:
    # # - perform simulation steps until Webots is stopping the controller
    # def cartesianConvertCompassBearingToHeading(heading):
    #     # /* 
    #     # * in webots, heading increasement is rotate in clockwise
    #     # * in cartesian, heading increasement is rotate in counterclockwise
    #     # * so headingInCartesian = 360-headingInWebots
    #     # * */
    #     heading = 360-heading
        
    #     # /* 
    #     # * in webots, heading is 0 if robot faced to y+ axis
    #     # * in cartesian, heading is 0 if robot face to x+ axis
    #     # * so headingInCartesian = headingInWebots+90
    #     # * */
    #     heading = heading + 90
    #     if (heading > 360.0):
    #         heading = heading - 360.0

    #     return heading


    # def positioningControllerGetRobotCoordinate():
    #     position = np.zeros(2)
    #     position[0] = (trans_field_array[0].getSFVec3f())[0]
    #     position[1] = -(trans_field_array[0].getSFVec3f())[2]
        
    #     return position   
        
    # def cartesianIsCoordinateEqual(coordinate1, coordinate2):
    #     if (abs(coordinate1[0]-coordinate2[0]) < COORDINATE_MATCHING_ACCURACY and abs(coordinate1[1]-coordinate2[1]) < COORDINATE_MATCHING_ACCURACY):
    #         return True
    #     else:
    #         return False
    
    # def cartesianIsThetaEqual(theta, theta2):
    #     if (abs(theta - theta2) < THETA_MATCHING_ACCURACY):
    #         return True
    #     else:
    #         return False
            
    # def get_bearing_in_degrees():
    #     north = compass.getValues()
    #     rad = math.atan2(north[1], north[0])
    #     bearing = (rad - 1.5708) / math.pi * 180.0
    #     if (bearing < 0.0):
    #         bearing = bearing + 360.0
    #     return bearing


    # def positioningControllerGetRobotHeading():
    #     return cartesianConvertCompassBearingToHeading(get_bearing_in_degrees())
        
    # def cartesianCalcDestinationThetaInDegrees(currentCoordinate,  destinationCoordinate):
    #     return math.atan2(destinationCoordinate[1] - currentCoordinate[1], destinationCoordinate[0] - currentCoordinate[0]) * 180 / math.pi


    # def cartesianCalcThetaDot(heading, destinationTheta):
    #     theta_dot = destinationTheta - heading;

    #     if (theta_dot > 180):
    #         theta_dot = -(360-theta_dot)
    #     elif (theta_dot < -180):
    #         theta_dot = (360+theta_dot)

    #     return theta_dot

    # def cartesianCalcDistance(currentCoordinate, destinationCoordinate):
    #     return math.sqrt(math.pow(destinationCoordinate[0]-currentCoordinate[0], 2) + math.pow(destinationCoordinate[1]-currentCoordinate[1], 2))


    # def positioningControllerCalcDistanceToDestination(destinationCoordinate):

    #     currentCoordinate = positioningControllerGetRobotCoordinate()
    #     return cartesianCalcDistance(currentCoordinate, destinationCoordinate)

    # def positioningControllerCalcThetaDotToDestination(destinationCoordinate):

    #     currentCoordinate = positioningControllerGetRobotCoordinate()
    #     robotHeading = positioningControllerGetRobotHeading()
    #     destinationTheta = cartesianCalcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate)
    #     return cartesianCalcThetaDot(robotHeading, destinationTheta)



    # def moveToDestination(destinationCoordinate):
    #     currentCoordinate = positioningControllerGetRobotCoordinate()
    #     print("Initial Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1])
    #     print("Destination Coordinate: %.5f %.5f\n", destinationCoordinate[0], destinationCoordinate[1])
        
    #     # if the robot is already at the destination location
    #     if (cartesianIsCoordinateEqual(positioningControllerGetRobotCoordinate(), destinationCoordinate)):
            
    #             printf("Robot is already at the destination location\n")
    #             return
            
    #     # thetaDot is the degree of rotation needed by the robot to face the destination
    #     # thetaDot is zero if robot is facing the destination

    #     thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate)
    #     print("thetaDotToDestination: %.5f\n", thetaDotToDestination)

    #     rotateHeading(thetaDotToDestination)

    #     # the distance needed for the robot to reach its destination
    #     distanceToDestination = positioningControllerCalcDistanceToDestination(destinationCoordinate)
    #     print("distanceToDestination: %.5f\n", distanceToDestination)
        
    #     moveForward(distanceToDestination)
        
    #     currentCoordinate = positioningControllerGetRobotCoordinate()
    #     print("Stop Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1])


