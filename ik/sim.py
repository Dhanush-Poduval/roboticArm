import pybullet as p 
import os
import time 
import pybullet_data
import math 

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
urdf_path = os.path.join(parent_dir, 'four_dof_arm.urdf')
#debug for not loading urdf
print(f"Final URDF Path: {urdf_path}") 
if not os.path.exists(urdf_path):
    print("Not found")
    exit()
physicsClient = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # 
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf") 
robot_arm = p.loadURDF(urdf_path, useFixedBase=True, basePosition=[0,0,0])
EE_LINK_INDEX = 3 
num_joints = p.getNumJoints(robot_arm)

print(f"Robot Loaded with {num_joints} joints. End-Effector Index: {EE_LINK_INDEX}")
target_pos = [0.0, 0.7, 1.0] 

target_orientation = p.getQuaternionFromEuler([0, -math.pi / 4, 0]) 
print("\nRunning Inverse Kinematics")
joint_poses = p.calculateInverseKinematics(
    bodyUniqueId=robot_arm,
    endEffectorLinkIndex=EE_LINK_INDEX,
    targetPosition=target_pos,
    maxNumIterations=50,
    residualThreshold=1e-4
)
for i in range(num_joints):
    joint_type = p.getJointInfo(robot_arm, i)[2]
    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        p.setJointMotorControl2(
            bodyUniqueId=robot_arm,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_poses[i]
        )

print(f"Target Position: {target_pos}")
print(f"Calculated Joint Poses (rad): {[round(angle, 3) for angle in joint_poses]}")
print("Starting Simulation to move the arm")
sim_time_seconds = 5
for i in range(sim_time_seconds * 240):
    p.stepSimulation()
    time.sleep(1. / 240.) 
p.disconnect()
print("\nSimulation Finished. Arm should be at the target.")
