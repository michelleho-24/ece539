import math
import numpy as np
import random

class rrt_limited: 
    def __init__(self, obstacles, sample_radius):
        self.obstacles = obstacles #Array of obstacles in format (x, y, z, r) treated as circles
        self.sample_radius = sample_radius

    def sample_random(self, curr_pos): 
        
        print(curr_pos)
        # Using this: https://stackoverflow.com/questions/5837572/generate-a-random-point-within-a-circle-uniformly
        r = self.sample_radius * math.sqrt(random.uniform(0,1))
        theta = random.uniform(0,1) * 2 * math.pi
        x = curr_pos[0] + r * math.cos(theta)
        y = curr_pos[2] + r * math.sin(theta)
        
        sample_point = np.array([x, curr_pos[1], y])
        print(sample_point)
        return sample_point
    
    def check_free(self, sample):
        flag = True
        for obs in self.obstacles:
            distance = np.sqrt(((sample[0]-obs[0])**2)+((sample[2]-obs[2])**2))
            if distance <= obs[2]:
                flag = False
                break
        return flag

    def sample_random_free(self, curr_pos):
        sample = self.sample_random(curr_pos)
        while not (self.check_free(sample)):
            sample = self.sample_random(curr_pos)

        return sample #returns a (x, y, z) value in webots space 
                
    def edge_free(self, curr_pos, sampleB):
        Ax = curr_pos[0]
        Ay = curr_pos[2]
        Bx = sampleB[0]
        By = sampleB[2]
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
                if d > radius:
                    flag = True 
        return flag

    def expand_rrt(self, curr_pos):
        proposed_sample = self.sample_random_free(curr_pos)
        while not (self.edge_free(curr_pos, proposed_sample)):
            proposed_sample = self.sample_random_free(curr_pos)

        return proposed_sample
        
