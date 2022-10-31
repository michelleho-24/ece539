#LIBRARY IMPORTS
from gettext import translation
import math
import numpy as np


class Movement:
    def __init__(self, robot, motorL, motorR, compass, gyro):
        self.robot = robot
        self.motorL = motorL
        self.motorR = motorR
        self.compass = compass
        self.gyro = gyro
        #DEFINED CONSTANTS
    COORDINATE_MATCHING_ACCURACY = 0.01
    THETA_MATCHING_ACCURACY = 1
    # ANGULAR SPEED IN RADIANS 32778.0
    ROBOT_ANGULAR_SPEED_IN_DEGREES = 42
    #Linear velocity 0.0549999930
    TANGENSIAL_SPEED = 0.128
    MAX_SPEED = 10

    def motorStop(self):
        self.motorL.setVelocity(0)
        self.motorR.setVelocity(0)

    def motorMoveForward(self):
        self.motorL.setVelocity(self.MAX_SPEED)
        self.motorR.setVelocity(self.MAX_SPEED)

    def motorRotateLeft(self):
        self.motorL.setVelocity(-self.MAX_SPEED)
        self.motorR.setVelocity(self.MAX_SPEED)

    def motorRotateRight(self):
        self.motorL.setVelocity(self.MAX_SPEED)
        self.motorR.setVelocity(-self.MAX_SPEED)

    def moveForward(self, distance):
        # the duration required for the robot to move by the specified distance
        duration = distance / self.TANGENSIAL_SPEED
        print("duration to reach target location: %.5f\n", duration)

        #set robot motor to move forward
        self.motorMoveForward()

        #run the simulator
        start_time = self.robot.getTime()
        while True:
            if (self.robot.getTime() >= start_time + duration):
                break
            self.robot.step(8)
        #stop the motor
        self.motorStop()
        self.robot.step(8)

    def rotateHeading(self, thetaDot):
        if not (self.cartesianIsThetaEqual(thetaDot, 0)):
            duration = abs(thetaDot) / self.ROBOT_ANGULAR_SPEED_IN_DEGREES
            print("duration to face the destination: %.5f\n", duration)

            #if thetaDot > 0, robot will rotate to left
            if (thetaDot > 0):
                # set robot motor to rotate left
                self.motorRotateLeft()
            #if thetaDot < 0, robot will rotate to right
            elif (thetaDot < 0):
                # set robot motor to rotate right
                self.motorRotateRight()

            # run the simulator
            start_time = self.robot.getTime()
            while True:
                    #print(self.gyro.getValues())
                    if (self.robot.getTime() >= start_time + duration):
                        break
                    self.robot.step(8)

    #motorR.setPosition(0.1)
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    def cartesianConvertCompassBearingToHeading(self, heading):
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


    def positioningControllerGetRobotCoordinate(self, translation_array):
        position = np.zeros(2)
        position[0] = (translation_array[0])
        position[1] = -(translation_array[2])
        
        return position   
        
    def cartesianIsCoordinateEqual(self, coordinate1, coordinate2):
        if (abs(coordinate1[0]-coordinate2[0]) < self.COORDINATE_MATCHING_ACCURACY and abs(coordinate1[1]-coordinate2[1]) < self.COORDINATE_MATCHING_ACCURACY):
            return True
        else:
            return False
    
    def cartesianIsThetaEqual(self, theta, theta2):
        if (abs(theta - theta2) < self.THETA_MATCHING_ACCURACY):
            return True
        else:
            return False
            
    def get_bearing_in_degrees(self):
        north = self.compass.getValues()
        rad = math.atan2(north[1], north[0])
        bearing = (rad - 1.5708) / math.pi * 180.0
        if (bearing < 0.0):
            bearing = bearing + 360.0
        return bearing


    def positioningControllerGetRobotHeading(self):
        heading = self.get_bearing_in_degrees()
        return self.cartesianConvertCompassBearingToHeading(heading)
        
    def cartesianCalcDestinationThetaInDegrees(self, currentCoordinate,  destinationCoordinate):
        return math.atan2(destinationCoordinate[1] - currentCoordinate[1], destinationCoordinate[0] - currentCoordinate[0]) * 180 / math.pi


    def cartesianCalcThetaDot(self, heading, destinationTheta):
        theta_dot = destinationTheta - heading;

        if (theta_dot > 180):
            theta_dot = -(360-theta_dot)
        elif (theta_dot < -180):
            theta_dot = (360+theta_dot)

        return theta_dot

    def cartesianCalcDistance(self, currentCoordinate, destinationCoordinate):
        return math.sqrt(math.pow(destinationCoordinate[0]-currentCoordinate[0], 2) + math.pow(destinationCoordinate[1]-currentCoordinate[1], 2))


    def positioningControllerCalcDistanceToDestination(self, destinationCoordinate, curr_pos):

        currentCoordinate = self.positioningControllerGetRobotCoordinate(curr_pos)
        return self.cartesianCalcDistance(currentCoordinate, destinationCoordinate)

    def positioningControllerCalcThetaDotToDestination(self, destinationCoordinate, curr_pos):

        currentCoordinate = self.positioningControllerGetRobotCoordinate(curr_pos)
        robotHeading = self.positioningControllerGetRobotHeading()
        destinationTheta = self.cartesianCalcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate)
        return self.cartesianCalcThetaDot(robotHeading, destinationTheta)



    def moveToDestination(self, destinationCoordinate, curr_pos):
        currentCoordinate = self.positioningControllerGetRobotCoordinate(curr_pos)
        # print("Initial Coordinate: {2.5f} {2.5f}\n".format(currentCoordinate[0], currentCoordinate[1]))
        # print("Destination Coordinate: {2.5f} {2.5f}\n", destinationCoordinate[0], destinationCoordinate[1])
        
        # if the robot is already at the destination location
        if (self.cartesianIsCoordinateEqual(self.positioningControllerGetRobotCoordinate(curr_pos), destinationCoordinate)):
                print("Robot is already at the destination location\n")
                return
            
        # thetaDot is the degree of rotation needed by the robot to face the destination
        # thetaDot is zero if robot is facing the destination

        thetaDotToDestination = self.positioningControllerCalcThetaDotToDestination(destinationCoordinate, curr_pos)
        # print("thetaDotToDestination: %.5f\n", thetaDotToDestination)

        self.rotateHeading(thetaDotToDestination)

        # the distance needed for the robot to reach its destination
        distanceToDestination = self.positioningControllerCalcDistanceToDestination(destinationCoordinate, curr_pos)
        # print("distanceToDestination: %.5f\n", distanceToDestination)
        
        self.moveForward(distanceToDestination)
        
        currentCoordinate = self.positioningControllerGetRobotCoordinate(curr_pos)
        # print("Stop Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1])


