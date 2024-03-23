import environment as env
import pybullet as p
import time
import pybullet_data
import numpy as np
import kinematics as k
import RRT
import RRTStar
import RRTStarWrapped

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")

robot1 = p.loadURDF("KUKA_IIWA_URDF-master/iiwa14.urdf", [-0.1, 0, 0.18], useFixedBase=False, flags = p.URDF_USE_SELF_COLLISION,globalScaling = 0.5)
#robot2 = p.loadURDF("boxer/boxer.urdf",[-0.35, 0, 0.04],p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True, flags = p.URDF_USE_SELF_COLLISION)

#jointType = p.JOINT_FIXED
#p.createConstraint(robot1, 0, robot2, 0, jointType, [0,0,0], [0,0,0], [0,0,0])

kinematic = k.Kinematics(robot1) 

path = []

start = kinematic.getlinkState()
start_config = kinematic.inverseKinematics(start)
goal =   [0.29763578226262166, -0.222429701774353, 0.45323642959348415]

print("Calculating...........")
planningAlgo = RRTStarWrapped.RRTStar(robot1,start_config,goal).RRT()
print("planningAlgo:",planningAlgo)

path = planningAlgo



path_points = []

for config1 in path:

    for i in range(1,7):
        p.resetJointState(
            bodyUniqueId = robot1,
            jointIndex = i,
            targetValue = config1[i-1],
            targetVelocity = 0)
        
        p.stepSimulation()
        time.sleep(0.1)

    state = p.getLinkState(robot1,7)
    end_effector_pos = state[0]

    path_points.append(end_effector_pos)

    if len(path_points) >1 :
        p.addUserDebugLine(path_points[-2],path_points[-1],lineColorRGB = [1,0,0])

# Press 'q' to stop the simulation
while True:
    p.stepSimulation()
    time.sleep(1/5)
    
    keys = p.getKeyboardEvents()
    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
        break 

p.disconnect()