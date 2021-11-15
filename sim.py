import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import modern_robotics as mr
def Inertia_matrix(ixx,iyy,izz,ixy,iyz,ixz):
	I = np.array([[ixx ,ixy ,ixz],[ixy ,iyy ,iyz ],[ixz ,iyz ,izz ]])
	return I

## setup
useMaximalCoordinates = False
p.connect(p.GUI)
p.setGravity(0, 0, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

robot_id = p.loadURDF("humanoid/humanoid.urdf", [0, 0, 0.00],[0, 0, 0, 1])
numJoints = p.getNumJoints(robot_id)
p.resetBasePositionAndOrientation(robot_id, [0, 0, 0], [0, 0, 0, 1])
for i in range(0,numJoints):
	p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=0, force=0)

timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)

while p.isConnected():
	timeStep = p.readUserDebugParameter(timeStepId)
	p.setTimeStep(timeStep)
	dt = timeStep

	if (not useRealTimeSim):
		p.stepSimulation()
		time.sleep(timeStep)
