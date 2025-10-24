import pybullet as p 
import os
import time 
import pybullet_data
import math 
import numpy as np
from inverse_kinematics import inverse_kinematics

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
# Insert this right after loading the URDF:
print("\n--- PyBullet Joint Info ---")
for i in range(p.getNumJoints(robot_arm)):
    joint_info = p.getJointInfo(robot_arm, i)
    # joint_info[1] is the joint name (bytes), joint_info[2] is the joint type
    print(f"Index {i}: Name '{joint_info[1].decode('utf-8')}', Type: {joint_info[2]}")
print("---------------------------\n")
EE_LINK_INDEX = 3 
LINK_RADIUS=0.05
shelf_safety_margin=0.1
shelf_half=np.array([0.8,0.4,0.02])
shelf_pos=np.array([0.0, 0.6, 0.5])
num_joints = p.getNumJoints(robot_arm)
containers={}
rack_positions={
    'C01': (0.4, 0.7, 0.1), 
    'C02': (0.0, 0.7, 0.1),
    'C03': (-0.4, 0.7, 0.1),
}
shelf_positions={
    'Aruco_101': (-0.8, 0.0, 0.3),
    'Aruco_102': (0.8, 0.0, 0.3),
}
classification_results={
    'C01': 'Aruco_102', 
    'C02': 'Aruco_101', 
    'C03': 'Aruco_102',
}

def collision(robot_arm,joint_indices,joint_positions):
    p.resetJointStatesMultiDof(robot_arm,joint_indices,joint_positions)
    safeDistance=(shelf_half[1] + LINK_RADIUS + shelf_safety_margin)**2
    link2_state=p.getLinkState(robot_arm,1)
    link2_pos=np.array(link2_state[0])
    link3_state=p.getLinkState(robot_arm,2)
    link3_pos=np.array(link3_state[0])
    check_links=[link2_pos,link3_pos]
    for link_pos in check_links:
        dist=np.sum((link_pos - shelf_pos)**2)
        if dist<safeDistance:
            return True
    return False

def execute_pos(joint_target, robot_arm_id, duration_seconds=1.0, sim_time_per_step=1.0 / 240.0):
    num_steps = int(duration_seconds / sim_time_per_step)
    for i in range(num_steps):
        for joint_index in range(4):
         p.setJointMotorControl2(
            bodyUniqueId=robot_arm_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_target[joint_index]
         )
        p.stepSimulation()
        time.sleep(sim_time_per_step)

print(f"Robot Loaded with {num_joints} joints. End-Effector Index: {EE_LINK_INDEX}")
target_pos = [0.1, 0.3,0.5] 

target_orientation = p.getQuaternionFromEuler([0, -math.pi / 4, 0]) 
print("\nRunning Inverse Kinematics")
'''
joint_poses=inverse_kinematics(*target_pos,angle=0)
if joint_poses is not False:
    final_poses = np.array(joint_poses)
    print(f"Calculated Joint Poses (rad): {[round(angle, 3) for angle in final_poses]}")

    # 2. Execute Movement (Moves directly to the target pose)
    print("Executing direct movement to IK target...")
    execute_pos(final_poses, robot_arm, duration_seconds=2.0)
    
else:
    print("Mission failed: Analytical IK could not reach the target.")
'''
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

''''''
print(f"Target Position: {target_pos}")
print(f"Calculated Joint Poses (rad): {[round(angle, 3) for angle in joint_poses]}")
print("Starting Simulation to move the arm")
sim_time_seconds = 5
for i in range(sim_time_seconds * 240):
    p.stepSimulation()
    time.sleep(1. / 240.) 
p.disconnect()
print("\nSimulation Finished. Arm should be at the target.")
