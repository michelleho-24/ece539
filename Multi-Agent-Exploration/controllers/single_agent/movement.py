from gettext import translation
import math
import numpy as np


class Movement:
    def __init__(self, robot, robot_node, motorL, motorR, compass, gyro, imu, lDist, rDist):
        self.robot = robot
        self.robot_node = robot_node
        self.motorL = motorL
        self.motorR = motorR
        self.compass = compass
        self.gyro = gyro
        self.imu = imu 
        self.lDist = lDist
        self.rDist = rDist
    
    #DEFINED CONSTANTS
    COORDINATE_MATCHING_ACCURACY = 0.01
    THETA_MATCHING_ACCURACY = 5
    # ANGULAR SPEED IN RADIANS 1.3
    ROBOT_ANGULAR_VELOCITY_IN_RADIANS = 2.57107753376862 #2.92107753376862
    ROBOT_ANGULAR_SPEED_IN_DEGREES = ROBOT_ANGULAR_VELOCITY_IN_RADIANS * (180/math.pi)
    #Linear velocity 0.0549999930
    TANGENSIAL_SPEED = 0.054999784124110815
    MAX_SPEED = 10
    CLOSE_DISTANCE = 60

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
        #print("duration to reach target location: ", duration)

        #set robot motor to move forward
        self.motorMoveForward()

        #run the simulator
        start_time = self.robot.getTime()
        while True:
            if (self.robot.getTime() >= start_time + duration):
                self.motorStop()
                self.robot.step(8)
                trans_field = self.robot_node.getField("translation")
                position = trans_field.getSFVec3f()
                return position
            if (self.lDist.getValue() < self.CLOSE_DISTANCE or self.rDist.getValue() < self.CLOSE_DISTANCE):
                self.motorStop()
                self.robot.step(8)
                trans_field = self.robot_node.getField("translation")
                position = trans_field.getSFVec3f()
                return position
            self.robot.step(8)

    def rotateHeading(self, thetaDot):
        if not (self.cartesianIsThetaEqual(thetaDot, 0)):
            duration = abs(thetaDot) / self.ROBOT_ANGULAR_SPEED_IN_DEGREES
            #print("duration to face the destination: ", duration)

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
            self.motorStop()

    #motorR.setPosition(0.1)

    def positioningControllerGetRobotCoordinate(self):
        trans_field = self.robot_node.getField("translation")
        translation_array = trans_field.getSFVec3f()
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
            
    def get_heading(self):
        # north = self.compass.getValues()
        # rpy = self.imu.getRollPitchYaw()
        # north = rpy[2]
        # print("angle", north)

        heading = self.imu.getRollPitchYaw()[2]/math.pi * 180.0 - 90
        if (heading < 0):
            heading = heading + 360
        #print("Heading: ", heading)

        # rad = math.atan(north[2]/ north[0]) #0, 2
        # # bearing = (rad - 1.5708) / math.pi * 180.0
        # bearing = rad / math.pi * 180.0
        # if (bearing < 0.0):
        #     bearing = bearing + 180 #360
        # print("Rad: ", rad)
        return heading


    def positioningControllerGetRobotHeading(self):
        heading = self.get_bearing_in_degrees()
        return heading
        #return self.cartesianConvertCompassBearingToHeading(heading)
        
    def cartesianCalcDestinationThetaInDegrees(self, currentCoordinate,  destinationCoordinate):
        #print("Destination Theta: ", math.atan2(destinationCoordinate[0] - currentCoordinate[0], destinationCoordinate[1] - currentCoordinate[1]) * 180 / math.pi)
        # print("currentCoordinate", currentCoordinate)
        # print("destinationcoordinate", destinationCoordinate)
        a = [0, 0.5]
        b = [destinationCoordinate[0] - currentCoordinate[0], destinationCoordinate[1] - currentCoordinate[1]]
        # print("A: " ,a)
        # print("B: ", b)
        abdot = np.dot(a, b)
        abmag = np.linalg.norm(a)*np.linalg.norm(b)
        # print("Dot a,b: ", np.dot(a, b))
        # print("Mag a,b: ", np.linalg.norm(a)*np.linalg.norm(b))
        # print("arccos: ", math.acos(abdot/abmag) * 180/math.pi)
        # print("NEW THETA: ", math.acos(np.dot(a, b) / np.linalg.norm(a)*np.linalg.norm(b)) * 180/math.pi)

        # currentCoordinate [-0.229753 -0.031428]
        # destinationcoordinate [-0.344, 0.0106, 0.0819]
        # radians = math.atan((currentCoordinate[1] - destinationCoordinate[1] ) / (currentCoordinate[0] - destinationCoordinate[0] ))
        # radians = radians * 180/math.pi
        degrees =  math.acos(abdot / abmag) * 180/math.pi
        if (destinationCoordinate[0] > currentCoordinate[0]):
            degrees = -degrees
        # print("Destination Degree: ", degrees)
        # if (radians < 0):
        #     radians = radians + 360
        # print("Target Angle: ", radians)

        return degrees
        # print("radians", math.atan2(destinationCoordinate[0] - currentCoordinate[0], -(destinationCoordinate[1] - currentCoordinate[1])))

       #  return math.atan2(destinationCoordinate[0] - currentCoordinate[0], -(destinationCoordinate[1] - currentCoordinate[1])) * 180 / math.pi
        # return math.atan2(destinationCoordinate[1] - currentCoordinate[1], destinationCoordinate[0] - currentCoordinate[0]) * 180 / math.pi


    def cartesianCalcThetaDot(self, heading, destinationTheta):
        # heading = 360
        theta_dot = destinationTheta - heading

        # print("heading", heading)
        # print("destination theta", destinationTheta)

        if (theta_dot > 180):
            theta_dot = -(360-theta_dot)
        elif (theta_dot < -180):
            theta_dot = (360+theta_dot)
        # print("theta dot", theta_dot)
        return theta_dot

    def cartesianCalcDistance(self, currentCoordinate, destinationCoordinate):
        return math.sqrt(math.pow(destinationCoordinate[0]-currentCoordinate[0], 2) + math.pow(destinationCoordinate[1]-currentCoordinate[1], 2))


    def positioningControllerCalcDistanceToDestination(self, destinationCoordinate, curr_pos):

        currentCoordinate = self.positioningControllerGetRobotCoordinate()
        return self.cartesianCalcDistance(currentCoordinate, destinationCoordinate)

    def positioningControllerCalcThetaDotToDestination(self, destinationCoordinate, curr_pos):

        currentCoordinate = self.positioningControllerGetRobotCoordinate()
        robotHeading = self.get_heading()
        # print("Robot Heading: ", robotHeading)
        destinationTheta = self.cartesianCalcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate)
        return self.cartesianCalcThetaDot(robotHeading, destinationTheta)


    '''
    Takes in destinationCoordinate = [x, y, z]
    and curr_pos[x, y, z]
    '''
    def moveToDestination(self, destinationCoordinate, curr_pos):
        print(curr_pos)
        destinationCartesian = np.zeros(2)
        destinationCartesian[0] = destinationCoordinate[0]
        destinationCartesian[1] = -destinationCoordinate[2]
        destinationCoordinate = destinationCartesian
        

        currentCoordinate = self.positioningControllerGetRobotCoordinate()
        # print("Initial Coordinate:", currentCoordinate)
        # print("Destination Coordinate: ", destinationCoordinate)
        
        # if the robot is already at the destination location
        if (self.cartesianIsCoordinateEqual(self.positioningControllerGetRobotCoordinate(), destinationCoordinate)):
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
        
        final_destination = self.moveForward(distanceToDestination)
        
        # currentCoordinate = self.positioningControllerGetRobotCoordinate(curr_pos)
        # print("Final Coordinate:", currentCoordinate)

        self.rotateHeading(-thetaDotToDestination)
        #print("Stop Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1])

        return final_destination