import pybullet as p 
import os
import time 
import pybullet_data
import math 
import numpy as np
from ..inverse_k import inverse_kinematics

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
urdf_path = os.path.join(parent_dir, 'four_dof_arm.urdf')
print(f"Final URDF Path: {urdf_path}") 
if not os.path.exists(urdf_path):
    print("Not found")
    exit()
physicsClient = p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0, 0, 0)
p.loadURDF("plane.urdf") 
robot_arm = p.loadURDF(urdf_path, useFixedBase=True, basePosition=[0,0,0])
print("\nPyBullet Joint Info")
for i in range(p.getNumJoints(robot_arm)):
    joint_info = p.getJointInfo(robot_arm, i)
EE_LINK_INDEX = 3 
LINK_RADIUS=0.05
L0_BASE_HEIGHT = 0.1
shelf_safety_margin=0.1
shelf_half=np.array([0.8,0.4,0.02])
shelf_pos=np.array([0.0, 1.1, 0.37])
num_joints = p.getNumJoints(robot_arm)
containers={}

shelf_top_z=shelf_pos[2]+shelf_half[2]
container_base_z=shelf_top_z

rack_positions={
    'C07': (-0.4, 0.7, container_base_z), 
    'C08': (0.0, 0.7, container_base_z),
    'C09': (0.4, 0.7, container_base_z),

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
LIFT_HEIGHT_Z=0.35
JOINT_INDICES = list(range(4))

'''
def load_shelf(position,half_extents):
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[2, 0.5, 0.5, 1])
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    shelf_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
    )
    print(f"Loaded shelf at location ${position}")
    return shelf_id
'''

def load_container(name,position):
    container_half_extents = [0.05, 0.05, 0.05]
    color = [np.random.rand(), np.random.rand(), np.random.rand(), 1]
    visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=container_half_extents, rgbaColor=color)
    collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=container_half_extents)
    
    
    base_z = position[2] + container_half_extents[2] 
    
    container_id = p.createMultiBody(
        baseMass=0.1, 
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=[position[0], position[1], base_z],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
    )
    print(f"Loaded Container {name} at: {[position[0], position[1], base_z]}")
    return container_id
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
#main_shelf_id=load_shelf(shelf_pos,shelf_half)
for name,pos in rack_positions.items():
   containers[name]=load_container(name,pos)

print(f"Robot Loaded with {num_joints} joints. End-Effector Index: {EE_LINK_INDEX}")


target_pos_tip = [0.0,0.85,0.39] 
length_EE=0.05
target_pos_ik = [target_pos_tip[0], target_pos_tip[1], target_pos_tip[2] -length_EE-L0_BASE_HEIGHT]

TARGET_MARKER_RADIUS = 0.02
target_marker_visual = p.createVisualShape(
    p.GEOM_SPHERE, 
    radius=TARGET_MARKER_RADIUS, 
    rgbaColor=[1, 0, 0, 1]  
)
target_marker_id = p.createMultiBody(
    baseMass=0,  
    baseCollisionShapeIndex=-1, 
    baseVisualShapeIndex=target_marker_visual,
    basePosition=target_pos_tip
)

clearance_pos=[0.3, 0.4,0.6]
APPROACH_HEIGHT_OFFSET = 0.1
approach_pos_tip = [target_pos_tip[0], target_pos_tip[1], target_pos_tip[2] -APPROACH_HEIGHT_OFFSET]
approach_pos_ik = [approach_pos_tip[0], approach_pos_tip[1], approach_pos_tip[2] -length_EE-L0_BASE_HEIGHT]


print(f'moving to clearence pos ${clearance_pos}')

joint_poses_clearence=inverse_kinematics(*clearance_pos,angle=0)
if joint_poses_clearence is not False:
    initial_joint_angles = np.array([p.getJointState(robot_arm, i)[0] for i in range(4)])
    
    best_pose = None
    min_dist = float('inf')
    for pose in joint_poses_clearence:
        pose_array = np.array(pose)
        distance = np.sum((pose_array - initial_joint_angles)**2)
        if distance < min_dist:
            min_dist = distance
            best_pose = pose
            
    if best_pose is not None:
        final_poses_clearance = np.array(best_pose) 
        print(f"Calculated Clearance Poses (rad): {[round(angle, 3) for angle in final_poses_clearance]}")
        execute_pos(final_poses_clearance, robot_arm, duration_seconds=2.0)
    else:
        print("Not succesfull")
        p.disconnect()
        exit()
else:
    print("Clearance is out of reach ")
    p.disconnect()
    exit()

current_joint_angles_np = np.array([p.getJointState(robot_arm, i)[0] for i in range(4)])
print(f"moving to approach position:{approach_pos_tip}")
joint_poses_approach=inverse_kinematics(*approach_pos_ik,angle=0)
final_approach_poses=None

if joint_poses_approach is not False:
      best_pose_approach = None
      min_dist = float('inf') 
      for i, pose in enumerate(joint_poses_approach): 
            formatted_angles = [round(angle, 3) for angle in pose]
            print(f"  Solution {i+1}: {formatted_angles}")
            pose_array = np.array(pose)
            distance = np.sum((pose_array - current_joint_angles_np)**2)

            if distance < min_dist:
                min_dist = distance
                best_pose_approach = pose 
      
      if best_pose_approach is not None:
          final_approach_poses = np.array(best_pose_approach)
          print(f"Chosen Approach Poses (rad): {[round(angle, 3) for angle in final_approach_poses]}")
          execute_pos(final_approach_poses, robot_arm, duration_seconds=1.0) 
      else:
          print("Could not get a valid pose after filtering.")
          
else:
   print("Target out of reach ")
if final_approach_poses is not None:
    current_joint_angles_np = final_approach_poses
    
    print(f"moving to final target position:{target_pos_tip}")
    
    joint_poses_final=inverse_kinematics(*target_pos_ik,angle=0)
    final_target_poses=None
    
    if joint_poses_final is not False:
          best_pose_final = None
          min_dist = float('inf') 
          for i, pose in enumerate(joint_poses_final):

                formatted_angles = [round(angle, 3) for angle in pose]
                print(f"  Solution {i+1}: {formatted_angles}")
                pose_array = np.array(pose)
                distance = np.sum((pose_array - current_joint_angles_np)**2)

                if distance < min_dist:
                    min_dist = distance
                    best_pose_final = pose 
          if best_pose_final is not None:
              theta2_final = best_pose_final[1]
              theta3_final = best_pose_final[2]
              theta4_required = -1 * (theta2_final + theta3_final)
              
              corrected_pose = list(best_pose_final)
              corrected_pose[3] = theta4_required 
              
              final_target_poses = np.array(corrected_pose)
              print(f"Chosen Final Poses (rad): {[round(angle, 3) for angle in final_target_poses]}")
              
          else:
              print("Could not get a valid pose after filtering.")
              
    else:
       print("Target out of reach ")
         
else:
   final_target_poses = None


print("\nRunning Inverse Kinematics")

'''joint_poses=inverse_kinematics(*target_pos,angle=0)
if joint_poses is not False:
    final_poses = np.array(joint_poses)
    print(f"Calculated Joint Poses (rad): {[round(angle, 3) for angle in final_poses]}")

    # 2. Execute Movement (Moves directly to the target pose)
    print("Executing direct movement to IK target...")
    execute_pos(final_poses, robot_arm, duration_seconds=2.0)
    
else:
    print("Mission failed: Analytical IK could not reach the target.")
'''
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

'''

if final_target_poses is not None:
    print("\nExecuting movement to selected IK target")
    execute_pos(final_target_poses, robot_arm, duration_seconds=1.0) 
    print("\nMovement Complete. Running final simulation loop.")
    for i in range(1 * 240): 
        p.stepSimulation()
        time.sleep(1. / 240.) 
    
else:
    print("\nMission Aborted: No valid final target pose available.")

p.disconnect()
print("\nSimulation Finished. Arm should be at the target.")