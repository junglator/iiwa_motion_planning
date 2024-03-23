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
    def __init__(self, config, length = float('inf')):
          self.config = config  # The robot configuration (joint angles) (dimension 6 integer list)
          self.parent = None  # Reference to the parent node in the tree (another Node)
          self.length = length  # The cost (or length) from the start node to this node (integer)

#This class defines all functions used in RRT*
class RRT:
     
    #Constructor
    def __init__(self, robotId, curPos, goal, n=7500, stepsize=0.5, goal_prob=0.1):
          self.robotId = robotId #Robot ID
          self.kinematic = K.Kinematics(self.robotId) #Kinematics of the Robot  
          self.start = Node(list(curPos[:6]), 0) #Initialize star node (length is 0)
          self.goal = goal  #Goal node
          self.iteration = min(n, 100000) #Number of iterations  
          self.stepsize = stepsize  #Step size (used while steering)
          self.interp_step = 0.3
          self.goal_prob = goal_prob #Goal probability (how much error we are willing to tolerate)
          self.radius = 3 #Radius length to check for neighbours  
          self.count = 0 #Counter (for debugging)  
          self.weight = [1,1,1,1,1,1] #Weights to calculate the cost function 
          self.obj = env.Environment().getObjects()
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
    
    #Find the nearest neighbour to "sample" in Tree
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
   
    #Cost function to check nearest neighbour (Euclidian distance)
    def euclidian_cost(self,q1,q2,weight):
        
        q = np.array(q1) - np.array(q2)
        
        cost = np.sqrt((weight*q*q).sum())
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
        length_increment = self.euclidian_cost(q_steered_config,nearestNeighbour.config,self.weight)
        q_steered.length = nearestNeighbour.length + length_increment

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
                  is_collision_free = False
                  break
              else:
                   pass
                #   print("No collision")
                  
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
          
          #Find position of final link (end-effector)    
          curPos = self.kinematic.getlinkState()
          
          #Find distace of end-effector from goal
          norm = np.linalg.norm(np.array(curPos) - np.array(goal))
          
          #Check if end-effector is within goal acceptable region
          if norm <= self.goal_prob:
               print("YAY!!! At Goal 🥳 .....")
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
    
    #main function
    def RRT(self):
         
         path = []
         
         #Initialize the tree with only one node (start node)
         Tree = [self.start]  
         
         #Start iterating the RRT* algorithm (with max iterations = self.iteration)
         for i in range(self.iteration):
             
             #Update count
            #  self.count += 1
             #Output count and present length of tree
            #  print("count: ", self.count)
             
             #Randomly sample a configuration (6-element list)
             q_rand = self.randomSampling() #Is list
             
             #Find the nearest neighbour to q_rand
             q_nearest = self.findNearestNeighbour(q_rand, Tree)  
             
             #Incrementally steer towards random sample (find q_new)
             #Presently, the parent is not known and the length is set to inf
             q_new = self.steer(q_rand, q_nearest, self.stepsize)
             
             #Check whether motion from q_nearest to q_new is collision-free
             #Otherwise discard the present iteration
             if not self.isCollisionFree(q_nearest.config, q_new.config):
                 continue
             
                #Find the nearest neighbours of interest and choose parent based on lowest cost
             else:
                                  
                 #Add q_new to tree
                 Tree.append(q_new)

             #Check if q_new is goal
             if self.checkGoal(self.goal, q_new) == True:  # Check if the goal is reached
                 path = self.updatePath(q_new)
                #  print("Goal found")
                 break

         if len(path) == 0:
             print("Alas, no optimal path found, try increasing number of iterations or check the coordinates")
             return "Alas, no optimal path found, try increasing number of iterations or check the coordinates" 
         else:
             print(path)
             return path