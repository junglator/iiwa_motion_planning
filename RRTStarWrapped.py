import math 
import pybullet as p
import time
import numpy as np
import random
import kinematics as K
import environment as env

#This class basically defines each node on the graph by its position, parent and cost (length)
class Node:
    #Constructor
    def __init__(self, config, tree_index = float("inf")):
          self.config = config  # The robot configuration (joint angles) (dimension 6 integer list)
          self.parent = None  # Reference to the parent node in the tree (another Node)
          self.index = tree_index  # The cost (or length) from the start node to this node (integer)

#This class defines all functions used in RRT*
class RRTStar:
    
    def __init__(self, robotId, curPos, goal, n=7500, stepsize=0.5, goal_prob=0.1):
          self.robotId = robotId #Robot ID
          self.kinematic = K.Kinematics(self.robotId) #Kinematics of the Robot  
          self.start = Node(list(curPos[:6]), 0) #Initialize star node (Index is 0)
          self.goal = goal  #Goal node
          self.iteration = min(n, 100000) #Number of iterations  
          self.stepsize = stepsize  #Step size (used while steering)
          self.interp_step = 0.15
          self.goal_prob = goal_prob #Goal probability (how much error we are willing to tolerate)
          self.radius = 1 #Radius length to check for neighbours  
          self.count = 0 #Counter (for debugging)  
          self.weight = [1,1,1,1,1,1] #Weights to calculate the cost function 
          self.obj = env.Environment().getObjects()
          self.obj = self.obj[0:3]
        #   self.RRT() #Auto-calling RRT so that it is implemented by itself  

    #Random sampling new configuration in robot joint space (returns list not node)
    def randomSampling(self):
          j1 = random.uniform(-3.14, 0)
          j2 = random.uniform(-2.09, 2.09)
          j3 = random.uniform(-3.14, 3.14)
          j4 = random.uniform(-2.09, 2.09)
          j5 = random.uniform(-3.14, 3.14)
          j6 = random.uniform(-3.14, 3.14)
          angles = [j1, j2, j3, j4, j5, j6]  # Set of joint angles
          return angles
    
    #Find the nearest neighbour to "sample" configuration in Tree
    def findNearestNeighbour(self, sample, Tree):
          
          #Initialize the cost and q_nearest by taking the start node            
          cost = self.euclidian_cost(sample,Tree[0].config,self.weight)
          q_nearest = Tree[0]
          
          #Execute if tree contains an element other than Start node
          if len(Tree) >1:
               for i in range(len(Tree)):
                    #Find the cost from the current element of the tree (temp)
                    temp = self.euclidian_cost(sample,Tree[i].config,self.weight)
                    
                    #If temp < cost, make the current node q_nearest
                    if temp < cost:
                         cost = temp
                         q_nearest = Tree[i]

          return q_nearest  
   
    #Cost function to find edge cost between 2 configurations (Euclidian distance)
    def euclidian_cost(self,q1,q2,weight):
        
        q = np.array(q1) - np.array(q2)
        
        cost = np.sqrt((weight*q*q).sum())
        # cost = np.sqrt((q*q).sum())
        return cost

    #Steering function: Outputs a node (q_near) which is incremental steering from nearest neighbour towards sample (q_rand)
    #sample is a config(list) whereas nearest neighbour is a node
    def steer(self, sample, nearestNeighbour, stepsize):
        
        #Direction vector from nearest neighbour to sample
        direction_vector = np.array(sample) - np.array(nearestNeighbour.config)
        
        #Norm of directio vector
        norm = np.linalg.norm(direction_vector)       
        #Taking into account the norm = zero case
        if norm == 0:
            norm = np.finfo(direction_vector.dtype).eps

        #Normalized direction vector
        normalized_vector = direction_vector / norm
        
        #Find the scaled vector (governed by step size unless the norm is actually < step size)
        scaled_vector  = np.minimum(stepsize, norm) * normalized_vector
        
        #Find the steered joint configuration
        q_steered_config = np.array(nearestNeighbour.config) + scaled_vector
        
        #Convert to node (for now, parent is not know, so the length is set to infinity)
        q_steered = Node(q_steered_config.tolist())
        
        #Assign length and parent to q_steered (this will be changed later while choosing neighbour)
        q_steered.parent = nearestNeighbour
        

        return q_steered
    
    # Boolean output whether the path between 2 configurations in collision free
    def isCollisionFree(self, start_config, fin_config):
          
          dvec = np.array(fin_config) - np.array(start_config)
          dvec_dist = np.linalg.norm(dvec)
          
          nsteps = dvec_dist // self.interp_step + 1
          vector_step = dvec/nsteps
          
          is_collision_free = True
          
          for i in range(1,int(nsteps)):
              
              check_config = start_config + i*vector_step
          
        
              if self.check_collision(check_config):
                  # print("Collision")
                  is_collision_free = False
                  break
              else:
                  # print("No collision")
                  lol = 1
                  
          return is_collision_free
                   

    #Checks for collision at given config
    def check_collision(self, config):
        
        #Set joint configuration to final node
        for i in range(1,7):
            p.resetJointState(
                bodyUniqueId = self.robotId,
                jointIndex = i,
                targetValue = config[i-1],
                targetVelocity = 0)
        
        p.stepSimulation()
        
        #Initialize collision detection Boolean
        collision_detected = False
        
        #Check for collision
        # Check for collision with each object in the environment
        for obj in self.obj:
              cont_pts = p.getContactPoints(self.robotId, obj)
              if len(cont_pts) > 0:
                  collision_detected = True 
                  break       
        
        return collision_detected
    
    
    #Check if current config (curConfig) is goal
    def checkGoal(self, goal, curConfig):
          
          #Set joint configuration to curConfig node
          for i in range(1,7):
              p.resetJointState(
                  bodyUniqueId = self.robotId,
                  jointIndex = i,
                  targetValue = curConfig.config[i-1],
                  targetVelocity = 0)
              p.stepSimulation()
          
          #Find position of final link (end-effector)    
          curPos = self.kinematic.getlinkState()
          # print(curPos)
          
          #Find distace of end-effector from goal
          norm = np.linalg.norm(np.array(curPos) - np.array(goal))
          
          # print(norm)
          
          #Check if end-effector is within goal acceptable region
          if norm <= self.goal_prob:
               print("YAY!!! At Goal 🥳 .....")
            #    print(norm)
               return True
          else:
               return False

    #Recursively generate path from start to current node
    def updatePath(self, q_new):
          path = []
          path.append(q_new.config)
          temp = q_new
          while temp.parent is not None:
               path.append(temp.parent.config)
               temp = temp.parent
          path.reverse()  # Reverse the path to start from the beginning
          
          return path
      
    #Find the nearest neighbours to 'q_new' in 'tree' within specified 'radius'
    def findNeighbours(self,Tree,q_new, radius=1):
          
          neigh = []
          # temp =[]
          
          for node in Tree:
              
              dist = np.linalg.norm(np.array(node.config) - np.array(q_new.config))
              
              if dist < radius:
                  neigh.append(node)
          
          # for i in Tree:
          #      temp.append(i.config)
          # kdtree = KDTree(temp)
          # search = kdtree.query_ball_point(q_new.config,radius)
          # for i in search:
          #      neigh.append(Tree[i])

          return neigh
      
    #Choose the best parent to q_new among neighbours based on cost
    def chooseParent(self, neighbors, q_new):
                     
          #Initialize best cost and best parent as current cost and parent
          best_cost = self.find_cost(q_new)
          best_parent = q_new.parent

          # Evaluate each neighbor as a potential parent
          for neighbor in neighbors:
                
              #Find cost
              cost = self.find_cost(neighbor) + self.euclidian_cost(neighbor.config,q_new.config,self.weight)
                
              #Update if new cost < best cost & No collision
              if cost < best_cost:
                  if self.isCollisionFree(neighbor.config,q_new.config):
                      best_cost = cost
                      best_parent = neighbor
                      # print("Parent change")
                
          q_new.parent = best_parent
          
          return 0
      
    #Find the cost of a node
    def find_cost(self,node):
        
        cost = 0;
        
        while node.index != 0:
            node_parent = node.parent
            cost = cost + self.euclidian_cost(node.config, node_parent.config,self.weight)
            node = node_parent
        
        return cost
          
    #Rewire the Tree if a better path is found
    def rewirePath(self, neigh, q_new):
              
          for neighbor in neigh:
              
              q_new_cost = self.find_cost(q_new)
              neighbor_cost = self.find_cost(neighbor)
              
              if self.euclidian_cost(neighbor.config,q_new.config,self.weight) + q_new_cost < neighbor_cost:
                  neighbor.parent = q_new
                  #print("path rewired")
                      
          return 0
      
    def cleanupPath(self,path):
        
        new_path = [path[0]]
        path_length = len(path)
        
        if path_length < 3:
            return path
        else:
            
            
            
            for i in range(2,len(path)):
                
                if not self.isCollisionFree(path[i], new_path[-1]):
                    new_path.append(path[i-1])
                    
                if i == (len(path)-1):
                    new_path.append(path[i])
            
            return new_path
        
    def costfromPath(self,path):

        cost = 0

        for i in range(1,len(path)):
            
            # cost_add = self.euclidian_cost(path[i-1], path[i], self.weight)
            cost_add = np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))
            cost += cost_add
        
        return cost
                
        
    
    #main function
    def RRT(self):
         
         
         goal_set = []
         
         #Initialize the tree with only one node (start node)
         tree_index = 0
         Tree = [self.start]  
         
         #Start iterating the RRT* algorithm (with max iterations = self.iteration)
         for i in range(self.iteration):
             
             #Update count
            #  self.count += 1
             #Output count and present length of tree
            #  print("count: ", self.count)
             # print("Tree len: ", len(Tree))
             
             #Randomly sample a configuration (6-element list)
             q_rand = self.randomSampling() #Is list
             
             #Find the nearest neighbour to q_rand
             q_nearest = self.findNearestNeighbour(q_rand, Tree)  
             
             #Incrementally steer towards random sample (find q_new)
             #Presently, the parent is q_nearest
             q_new = self.steer(q_rand, q_nearest, self.stepsize)
             # print(q_new.config)
             
             #Check whether motion from q_nearest to q_new is collision-free
             #Otherwise discard the present iteration
             if not self.isCollisionFree(q_nearest.config, q_new.config):
                 continue
             
                
             else:
                 
                 # If here, q_new is collision-free
                 
                 #First, add q_new to tree
                 tree_index += 1
                 q_new.index = tree_index
                 Tree.append(q_new)
                #  print("Tree length:",tree_index)
                 
                 # Now, we see if we can find a lower-cost parent for q_new
                 
                 #Step1: Find valid candidate neighbors
                 neighbours = self.findNeighbours(Tree,q_new,self.radius)
                 
                 # print("neighbors")
                 
                 # for node in neighbours:
                 #     print(node.config)
                 
                 #Step2: Choose parent from "neighbors"
                 self.chooseParent(neighbours,q_new)
                 
                 
                 #Rewire path if it makes sense for the neighbors
                 self.rewirePath(neighbours, q_new)


             #Check if q_new is goal, add to goal set
             if self.checkGoal(self.goal, q_new) == True:  # Check if the goal is reached
                 goal_set.append(q_new)
                         

             # #Check if q_new is goal
             # if self.checkGoal(self.goal, q_new) == True:  # Check if the goal is reached
                 
             #     if q_new.length < goal_path_cost:
             #         goal_path_cost = q_new.length
             #         path = self.updatePath(q_new)
             #         goal_config = q_new
             #         print("This Goal more optimal")
             #         print(path)
             #         break;
             #     else:
             #         print("This goal less optimal")
             #         print(path)
                 
                  
         

         if len(goal_set) == 0:
             print("Alas, no optimal path found, try increasing number of iterations or check the coordinates")
             return "Alas, no optimal path found, try increasing number of iterations or check the coordinates" 
         else:
             
             optimal_goal = None
             optimal_goal_cost = float("inf")
             optimal_path = None
             
             for goal in goal_set:
                 
                 goal_cost_old = self.find_cost(goal)
                #  print("Goal cost old:",goal_cost_old)
                 
                 # if goal_cost < optimal_goal_cost:
                 #     optimal_goal = goal
                 #     optimal_goal_cost = goal_cost
                     
                 # goal_path = self.updatePath(goal)
                 # print("original path: ",goal_path)
                 
                 # cleaned_path = self.cleanupPath(goal_path)
                 # print("cleaned up path: ",cleaned_path)
                 
                 # goal_cost_pre = self.find_cost(goal)
                 # print("pre-optimization Goal cost:",goal_cost_pre)
                 
                 # goal_path = self.updatePath(goal)
                 # cleaned_path = self.cleanupPath(goal_path)
                 # print("original path: ",goal_path)
                 # print("cleaned up path: ",cleaned_path)
                 
                 # #Change later
                 # goal_cost = self.find_cost(goal)
                 
                 # # goal_cost = self.costfromPath(cleaned_path)
                 # # print("post-optimization Goal cost:",goal_cost)               
                 
                 
                 # if goal_cost < optimal_goal_cost:
                 #     optimal_goal = goal
                 #     optimal_goal_cost = goal_cost
             
             for goal in goal_set:
                 
                 goal_path = []
                 goal_path = self.updatePath(goal)
                 cleaned_path = []    
                 cleaned_path = self.cleanupPath(goal_path)
                 # print("original path: ",goal_path)
                 # print("cleaned up path: ",cleaned_path)
                 goal_cost = 0
                 goal_cost = self.costfromPath(cleaned_path)
                 print("goal cost: ",goal_cost)
                 
                 if goal_cost < optimal_goal_cost:
                     optimal_goal = goal
                     optimal_goal_cost = goal_cost
                     optimal_path = cleaned_path
                 
                 
             
            #  print("optimal goal",optimal_goal.config)
            #  print("Optimal cost:", optimal_goal_cost)
            #  print("optimal path", optimal_path)
             
             #Set joint configuration to goal configuration
             for i in range(1,7):
                 p.resetJointState(
                     bodyUniqueId = self.robotId,
                     jointIndex = i,
                     targetValue = optimal_goal.config[i-1],
                     targetVelocity = 0)
                 p.stepSimulation()
                 
                 
             path = self.updatePath(optimal_goal)
             # print(path)
             
             return path
         
            
         
         