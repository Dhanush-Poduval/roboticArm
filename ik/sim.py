import pybullet
import time
import pybullet_data


physicsClient=pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

pybullet.loadURDF("plane.urdf")

robot_arm=pybullet.loadURDF('kuka_iiwa/model.urdf',useFixedBase=True)

while True:
    pybullet.stepSimulation()
    time.sleep(1./240.)