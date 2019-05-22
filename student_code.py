import math

def getIntersectionDistance(M, idx1, idx2):
    '''
       get the euclidean distance between intersection idx1 and idx2
    '''
    
    x1, y1 = M.intersections[idx1]
    x2, y2 = M.intersections[idx2]
    
    return math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

class PathNode():
    def __init__(self, idx, real_dist = 0, dist_to_goal = -1):
        self.idx = idx
        
        # the real distance incurred to reach this intersection
        self.real_dist = real_dist
        
        # the distance between this intersection and the goal intersection
        self.dist_to_goal = dist_to_goal
        
        self.total_cost = real_dist + dist_to_goal
        
        # A list containing child node
        self.children = []
        
        # A reference to the parent node
        self.parent = None
        
    def getPath(self):
        # get the path as a list of intersections from the start intersection till this node
        
        # intialize the list that will hold the path as a list of intersection numbers
        path = []
        
        node = self
        
        while(node):
            path.append(node.idx)
            node  = node.parent
            
        
        # return the reversed list since the start idx is at the end in 'path'
        return path[::-1]   
        
        
class PathTree():
    def __init__(self, start_idx):
        
        self.root = PathNode(start_idx)
        
    def addNode(self, 
                parent, 
                child_idx, 
                real_dist, 
                dist_to_goal):
        
        # create a Path Node for the child intersection
        child_node = PathNode(idx = child_idx,
                              real_dist = real_dist,
                              dist_to_goal = dist_to_goal)
        
        parent.children.append(child_node)
        child_node.parent = parent
        
        return child_node
        

       

class PriorityQueue():
    
    def __init__(self):
        
        self.members = []
        
    def push(self, path_node):
        
        # a flag to signal when the element is added in the following loop
        added = False
        
        # try to insert the 'path_node' at the appropriate positiion
        # in the members list
        for i, node in enumerate(self.members):
            
            if path_node.total_cost >= node.total_cost:
                self.members.insert(i, path_node)
                added = True
                break
              
        # if the 'added' flag is not set, it means that the current node
        # has the lowest cost, so add it as the highest priority
        # (the end of the members list)
        if not added:
            self.members.append(path_node)
                
            
    def pop(self):
        
        return self.members.pop()
    
        
def shortest_path(M,start,goal):
    
    print("shortest path called")
    
    
    # initialize a tree that will store the explored paths
    paths = PathTree(start)
    
    # get the node representing the root of the path tree as the current frontier
    frontier = PriorityQueue()
    frontier.push(paths.root)
    
    # get the lowest cost node from the frontier
    current_node = frontier.pop()
    
    # add the current node to the explored list
    explored =  set()
    explored.add(current_node.idx)
    
    # if this is already the goal node, return the path
    while(current_node and current_node.idx != goal):
    
        # get the intersections connected to the current frontier node intersection
        for idx in M.roads[current_node.idx]:

            # if the child intersection is not already explored, add it to the frontier
            if idx not in explored:

                new_node = paths.addNode(# the parent node: object of class PathNode
                                         parent = current_node, 
                                         # the child idx, this is an integer representing an intersection connected to the parent intersection
                                         child_idx = idx, 
                                         # the distance between the parent and the child intersection
                                         real_dist = current_node.real_dist + getIntersectionDistance(M, current_node.idx, idx), 
                                         # the birds eye distance to the goal (admissible estimate for A*)
                                         dist_to_goal = getIntersectionDistance(M, goal, idx)
                                         )

                frontier.push(new_node)
                
        # pop a node from the frontier and add it to the explored list
        current_node = frontier.pop()
        
        explored.add(current_node.idx)
    
    if current_node:
        # return the found path
        return current_node.getPath()
    else:
        # no path found
        return []