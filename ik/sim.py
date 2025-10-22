import pybullet
import time
import pybullet_data


physicsClient=pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

pybullet.loadURDF("plane.urdf")

robot_arm=pybullet.loadURDF('kuka_iiwa/model.urdf',useFixedBase=True)

print(pybullet.getNumJoints(robot_arm))

position,orientation=pybullet.getBasePositionAndOrientation(robot_arm)
print(position,orientation)

pybullet.getJointInfo(robot_arm,3)

pybullet.setGravity(0,0,-9.81)
pybullet.setRealTimeSimulation(1)
pybullet.setJointMotorControlArray(robot_arm,range(4),pybullet.POSITION_CONTROL,targetPositions=[0.2]*4)
while True:
    pybullet.stepSimulation()
    time.sleep(1./240.)