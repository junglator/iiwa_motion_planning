import pybullet as p
import time
import math
import numpy as np

"""
b3Printf: iiwa_link_ee
Joint Index: 0, Joint Name: world_iiwa_joint (fixed)
Joint Index: 1, Joint Name: iiwa_joint_1 (revolute)
Joint Index: 2, Joint Name: iiwa_joint_2 (revolute)
Joint Index: 3, Joint Name: iiwa_joint_3 (revolute)
Joint Index: 4, Joint Name: iiwa_joint_4 (revolute)
Joint Index: 5, Joint Name: iiwa_joint_5 (revolute)
Joint Index: 6, Joint Name: iiwa_joint_6 (revolute)
Joint Index: 7, Joint Name: iiwa_joint_7 (fixed)
Joint Index: 8, Joint Name: iiwa_joint_ee (fixed)
"""
class Kinematics:
    
    #Initialize robot parameters 
    def __init__(self,robotId):
        self.robot_id = robotId #robot ID
        self.jointindexarray = [1,2,3,4,5,6] #Indices of joints being used

    #Joint motion to a particular configuration (angle)
    def jointMotion(self,angle):
        
        p.setJointMotorControlArray(
                bodyUniqueId=self.robot_id,
                jointIndices = self.jointindexarray,
                controlMode=p.POSITION_CONTROL,
                targetPositions = angle,
                targetVelocities = [0,0,0,0,0,0],
                )
        
        return 0

    #Function returns current joint angles
    def getJointAngles(self):
        
        joint_angles = p.getJointStates(self.robot_id, self.jointindexarray)[0]
        
        return joint_angles

    #Inverse kinematics (only used for start configuration)
    def inverseKinematics(self,targetPosition):
        endEffectorIndex = 8
        joint_angles = p.calculateInverseKinematics(self.robot_id, endEffectorIndex, targetPosition)
        return joint_angles

    #Get position of end-effector link (to check for goal state)
    def getlinkState(self):
        link_index = 8
        link_state = p.getLinkState(self.robot_id,link_index)
        link_position = list(link_state[0])
        return link_position
    
    # def disableCollision(self):
    #     numJoints = p.getNumJoints(self.robot_id)
    #     # Disable collision for each joint
    #     for joint in range(numJoints):
    #         p.setCollisionFilterGroupMask(self.robot_id, joint, 0, 0)

    # def enableCollision(self):
    #     numJoints = p.getNumJoints(self.robot_id)
    #     # Disable collision for each joint
    #     for joint in range(numJoints):
    #         p.setCollisionFilterGroupMask(self.robot_id, joint, 1, 1)

    
            


