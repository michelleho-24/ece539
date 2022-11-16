import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple

def conf_free(q: np.ndarray, obstacles: List[Tuple[np.ndarray, float]]) -> bool:
    """
    Check if a configuration is in the free space.
    
    This function checks if the configuration q lies outside of all the obstacles in the connfiguration space.
    
    @param q: An np.ndarray of shape (2,) representing a robot configuration.
    @param obstacles: A list of obstacles. Each obstacle is a tuple of the form (center, radius) representing a circle.
    @return: True if the configuration is in the free space, i.e. it lies outside of all the circles in `obstacles`. 
             Otherwise return False.
    """
    x = q[0]
    y = q[1]
    flag = True 
    for obstacle in obstacles:
        center = obstacle[0]
        distance = np.sqrt(((y-center[1])**2)+((x-center[0])**2))
        if distance <= obstacle[1]: 
            flag = False 
            break 
    return flag

def edge_free(edge: Tuple[np.ndarray, np.ndarray], obstacles: List[Tuple[np.ndarray, float]]) -> bool:
    """
    Check if a graph edge is in the free space.
    
    This function checks if a graph edge, i.e. a line segment specified as two end points, lies entirely outside of
    every obstacle in the configuration space.
    
    @param edge: A tuple containing the two segment endpoints.
    @param obstacles: A list of obstacles as described in `config_free`.
    @return: True if the edge is in the free space, i.e. it lies entirely outside of all the circles in `obstacles`. 
             Otherwise return False.
    """
 # see attached file for intuition behind this logic 
 # line segment points
    A = edge[0]
    B = edge[1]
    Ax = A[0]
    Ay = A[1]
    Bx = B[0]
    By = B[1]
    flag = False
    for obstacle in obstacles:
        center = obstacle[0]
        h = center[0]
        k = center[1]
        radius = obstacle[1]
        
        # let E be the center of the obstacle 
        AEx = h - Ax
        AEy = h - Ay 
        ABx = Bx - Ax
        ABy = By - Ay
        BEx = h - Bx
        BEy = k - By
        
        AB_BE = ABx * BEx + ABy * BEy;
        AB_AE = ABx * AEx + ABy * AEy;
        
        # circle center closer to B 
        if AB_BE < 0:
            d = np.sqrt((k - By)**2 + (h - Bx)**2)
        # circle center closer to A 
        elif AB_AE < 0: 
            d = np.sqrt((k - Ay)**2 + (h - Ax)**2)
        # circle center in between A and B 
        else: 
            d = (np.abs((ABx)*(BEy)-(AEx)*(ABy))) / (np.sqrt(((ABx)**2)+((ABy)**2)))
        
        if conf_free(B, obstacles) & conf_free(A, obstacles):
            if d > radius:
                flag = True 
    return flag

def random_conf(width: float, height: float) -> np.ndarray:
    """
    Sample a random configuration from the configuration space.
    
    This function draws a uniformly random configuration from the configuration space rectangle. The configuration 
    does not necessarily have to reside in the free space.
    
    @param width: The configuration space width.
    @param height: The configuration space height.
    @return: A random configuration uniformily distributed across the configuration space.
    """
    x_rand = np.random.rand() * width
    y_rand = np.random.rand() * height 
    return np.array([x_rand, y_rand])

def random_free_conf(width: float, height: float, obstacles: List[Tuple[np.ndarray, float]]) -> np.ndarray:
    """
    Sample a random configuration from the free space.
    
    This function draws a uniformly random configuration from the configuration space
    rectangle that lies in the free space.
    
    @param width: The configuration space width.
    @param height: The configuration space height.
    @param obstacles: The list of configuration space obstacles as defined in `edge_free` and `conf_free`.
    @return: A random configuration uniformily distributed across the configuration space.
    """
    sample = random_conf(width, height)
    while not (conf_free(sample, obstacles)):
         sample = random_conf(width, height)
    return sample 

def nearest_vertex(conf: np.ndarray, vertices: np.ndarray) -> int:
    """
    Finds the nearest vertex to conf in the set of vertices.
    
    This function searches through the set of vertices and finds the one that is closest to 
    conf using the L2 norm (Euclidean distance).
    
    @param conf: The configuration we are trying to find the closest vertex to.
    @param vertices: The set of vertices represented as an np.array with shape (n, 2). Each row represents
                     a vertex.
    @return: The index (i.e. row of `vertices`) of the vertex that is closest to `conf`.
    """
    first_test = vertices[0]
    distance = np.Infinity
    index = 0
    i = 0 
    # loops through rows 
    for n in vertices:
        test_pt = n
        d_new = np.sqrt(((test_pt[0]-conf[0])**2) + ((test_pt[1]-conf[1])**2))
        if d_new < distance:
            distance = d_new 
            index = i
        i = i+1 
    return index 

def extend(origin: np.ndarray, target: np.ndarray, step_size: float=0.2) -> np.ndarray:
    """
    Extends the RRT at most a fixed distance toward the target configuration.
    
    Given a configuration in the RRT graph `origin`, this function returns a new configuration that takes a
    step of at most `step_size` towards the `target` configuration. That is, if the L2 distance between `origin`
    and `target` is less than `step_size`, return `target`. Otherwise, return the configuration on the line
    segment between `origin` and `target` that is `step_size` distance away from `origin`.
    
    @param origin: A vertex in the RRT graph to be extended.
    @param target: The vertex that is being extended towards.
    @param step_size: The maximum allowed distance the returned vertex can be from `origin`.
    
    @return: A new configuration that is as close to `target` as possible without being more than
            `step_size` away from `origin`.
    """
    # distance between origin and target
    d = np.sqrt(((origin[1]-target[1])**2) + ((origin[0]-target[0])**2))
    if d < step_size:  
        return target
    else:
        return np.array([(origin[0]+(step_size/d)*(target[0]-origin[0])),(origin[1]+(step_size/d)*(target[1]-origin[1]))])


def rrt(origin: np.ndarray, width: float, height: float, obstacles: List[Tuple[np.ndarray, float]],
        trials: int=1000, step_size: float=0.2) -> (np.ndarray, np.ndarray):
    """
    Explore a workspace using the RRT algorithm.
    
    This function builds an RRT using `trials` samples from the free space.
    
    @param origin: The starting configuration of the robot.
    @param width: The width of the configuration space.
    @param height: The height of the configuration space.
    @param obstacles: A list of circular obstacles.
    @param trials: The number of configurations to sample from the free space.
    @param step_size: The step_size to pass to `extend`.
    
    @return: A tuple (`vertices`, `parents`), where `vertices` is an (n, 2) `np.ndarray` where each row is a configuration vertex
             and `parents` is an array identifying the parent, i.e. `parents[i]` is the parent of the vertex in
             the `i`th row of `vertices.
    """
    num_verts = 1
    
    vertices = np.zeros((trials + 1, len(origin)))
    vertices[0, :] = origin
    
    parents = np.zeros(trials + 1, dtype=int)
    parents[0] = -1
    
    for trial in range(trials):
        #TODO: Fill this loop out for your assignment.
        sample = random_free_conf(width, height, obstacles) #Find a random config in the free space
        nearest_index = nearest_vertex(sample, vertices) 
        qs = extend(vertices[nearest_index], sample, step_size)
        if edge_free([vertices[nearest_index], qs], obstacles): 
            vertices[num_verts] = qs
            parents[num_verts] = nearest_index
            num_verts = num_verts + 1
    return vertices[:num_verts, :], parents[:num_verts]


def backtrack(index: int, parents: np.ndarray) -> List[int]:
    """
    Find the sequence of nodes from the origin of the graph to an index.
    
    This function returns a List of vertex indices going from the origin vertex to the vertex `index`.
    
    @param index: The vertex to find the path through the tree to.
    @param parents: The array of vertex parents as specified in the `rrt` function.
    
    @return: The list of vertex indicies such that specifies a path through the graph to `index`.
    """
    path = [index]
    p = parents[index]
    while p !=0: 
        path.append(p)
        p = parents[p]
    return path 


def main():
    width = 3
    height = 4

    obstacles = [(np.array([1, 1]), 0.25), (np.array([2, 2]), 0.25), (np.array([1, 3]), 0.3)]
    goal = (np.array([2.5, 3.5]), 0.25)

    origin = (0.1, 0.1)

    vertices, parents = rrt(origin, width, height, obstacles)

    index = nearest_vertex(goal[0], vertices)
    if np.linalg.norm(vertices[index, :] - goal[0]) < 0.25:
        print('Path found!')
        path_verts = backtrack(index, parents)
    else:
        print('No path found!')
        path_verts = []

    fig, ax = plt.subplots()

    ax.set_xlim([0, width])
    ax.set_ylim([0, height])
    ax.set_aspect('equal')

    for i in range(len(parents)):
        if parents[i] < 0:
            continue
        plt.plot([vertices[i, 0], vertices[parents[i], 0]], 
                [vertices[i, 1], vertices[parents[i], 1]], c='k')

    for i in path_verts:
        if parents[i] < 0:
            continue
        plt.plot([vertices[i, 0], vertices[parents[i], 0]], 
                [vertices[i, 1], vertices[parents[i], 1]], c='r')    

    for o in obstacles:
        ax.add_artist(plt.Circle(tuple(o[0]), o[1]))
        
    ax.add_artist(plt.Circle(tuple(goal[0]), goal[1], ec=(0.004, 0.596, 0.105), fc=(1, 1, 1)))

    plt.scatter([2.5], [3.5], zorder=3, c=np.array([[0.004, 0.596, 0.105]]), s=3)
    plt.scatter(vertices[path_verts, 0], vertices[path_verts, 1], c=np.array([[1, 0, 0]]), s=3, zorder=2)
    # plt.scatter(vertices[1:, 0], vertices[1:, 1], c=np.array([[0, 0, 0]]), s=3)

