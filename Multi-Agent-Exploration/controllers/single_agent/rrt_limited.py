import math
import numpy as np
import random
import csv
'''
This class takes in the current position of the robot in [x, y, z] 
and returns a coordinate in [x, y, z]
'''

class rrt_limited: 
    def __init__(self, obstacles, sample_radius, boundary_center, boundary_dim):
        self.obstacles = obstacles #Array of obstacles in format (x, y, z, r) treated as circles
        self.sample_radius = sample_radius
        self.boundary_center = boundary_center
        self.boundary_dim = boundary_dim

    def sample_random(self, curr_pos): 
        # Using this: https://stackoverflow.com/questions/5837572/generate-a-random-point-within-a-circle-uniformly
        r = self.sample_radius * math.sqrt(random.uniform(0,1))
        theta = random.uniform(0,1) * 2 * math.pi
        x = curr_pos[0] + r * math.cos(theta)
        y = curr_pos[2] + r * math.sin(theta)
        
        sample_point = np.array([x, curr_pos[1], y])

        return sample_point
    
    def check_free(self, sample):
        free = True
        for obs in self.obstacles:
            distance = np.sqrt(((sample[0]-obs[0])**2)+((sample[2]-obs[2])**2))
            if distance <= obs[3]: #Checks whether the point is inside an obstacle
                free = False
                break
        if ((sample[0] < (self.boundary_center[0]-self.boundary_dim[0])) or (sample[0] > (self.boundary_center[0]+self.boundary_dim[0]))) or ((sample[2] < (self.boundary_center[2]-self.boundary_dim[1])) or (sample[2] > (self.boundary_center[2]+self.boundary_dim[1]))):
                free = False
        return free

    def sample_random_free(self, curr_pos):
        sample = self.sample_random(curr_pos)
        while not (self.check_free(sample)):
            sample = self.sample_random(curr_pos)

        return sample #returns a (x, y, z) value in webots space 
                
    def edge_free(self, curr_pos, sampleB):
        '''
        Checks if the connecting edge between your current position and the sampled point do no collide with an obstacle
        '''
        Ax = curr_pos[0]
        Ay = curr_pos[2]
        #print("Current Pos: ", Ax, Ay)
        Bx = sampleB[0]
        By = sampleB[2]
        #print("Sampled Point: ", sampleB)
        flag = False

        for obstacle in self.obstacles:
            h = obstacle[0]
            k = obstacle[2]
            radius = obstacle[3]
            
            # let E be the center of the obstacle 
            AEx = h - Ax
            AEy = h - Ay 
            ABx = Bx - Ax
            ABy = By - Ay
            BEx = h - Bx
            BEy = k - By
            
            AB_BE = ABx * BEx + ABy * BEy
            AB_AE = ABx * AEx + ABy * AEy
            
            # circle center closer to B 
            if AB_BE < 0:
                d = np.sqrt((k - By)**2 + (h - Bx)**2)
            # circle center closer to A 
            elif AB_AE < 0: 
                d = np.sqrt((k - Ay)**2 + (h - Ax)**2)
            # circle center in between A and B 
            else: 
                d = (np.abs((ABx)*(BEy)-(AEx)*(ABy))) / (np.sqrt(((ABx)**2)+((ABy)**2)))
            
            if self.check_free(sampleB) and d > radius:
                flag = True
            else:
                # print("Sample intersecting wiht: ", sampleB)
                # print(obstacle)
                return False
        return flag

    def expand_rrt(self, curr_pos):
        
        proposed_sample = self.sample_random_free(curr_pos)
        while not (self.edge_free(curr_pos, proposed_sample)):
            proposed_sample = self.sample_random_free(curr_pos)
        
        #print("Proposed Sample: ", proposed_sample)
        return proposed_sample

    def rrt_iterate(self,start_pos):
        # begin by get next point
        new_sample = self.expand_rrt(start_pos)

        # if next point is not in radius of the goal

        rrt_sample_log = []
        for i in range(1,10): 
            new_sample =  self.expand_rrt(new_sample)
            rrt_sample_log.append(new_sample)
        
        print(rrt_sample_log)
        # Save experiment data
        with open('rrt_sample.txt', 'w') as f:
            # creating a csv writer object
            csvwriter = csv.writer(f)

            # writing the data rows``
            csvwriter.writerows(rrt_sample_log)
        f.close()

  
