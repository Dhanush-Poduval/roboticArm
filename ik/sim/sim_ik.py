import pybullet as p 
import os
import time 
import pybullet_data
import math 
import numpy as np
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
    print(f"Index {i}: Name '{joint_info[1].decode('utf-8')}', Type: {joint_info[2]}")

EE_LINK_INDEX = 3
LINK_RADIUS=0.05
shelf_safety_margin=0.1
shelf_half=np.array([0.8,0.4,0.02])
shelf_pos=np.array([0.0, 1.1, 0.37])
num_joints = p.getNumJoints(robot_arm)
containers={}

shelf_top_z=shelf_pos[2]+shelf_half[2]
container_base_z=shelf_top_z

rack_positions={
    'C07': (-0.4, 0.9, container_base_z), 
    'C08': (0.0, 0.9, container_base_z),
    'C09': (0.4, 0.9, container_base_z),
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
        baseCollisionShapeIndex=-1,
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

for name,pos in rack_positions.items():
   containers[name]=load_container(name,pos)

print(f"Robot Loaded with {num_joints} joints. End-Effector Index: {EE_LINK_INDEX}")

target_pos_tip = [-0.4,0.85,0.39] 

length_EE = 0.05


target_pos_ik = [target_pos_tip[0], target_pos_tip[1], target_pos_tip[2] + length_EE]

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

clearance_pos_tip = [0.0, 0.4, 0.7] 
clearance_pos_ik = [clearance_pos_tip[0], clearance_pos_tip[1], clearance_pos_tip[2] + length_EE]



print(f'\nmoving to clearence pos: {clearance_pos_tip}')

joint_poses_clearence_raw = p.calculateInverseKinematics(
    bodyUniqueId=robot_arm,
    endEffectorLinkIndex=EE_LINK_INDEX,
    targetPosition=clearance_pos_ik
) 
if joint_poses_clearence_raw:
    final_poses_clearance = np.array(joint_poses_clearence_raw[:4]) 

    theta2 = final_poses_clearance[1]
    theta3 = final_poses_clearance[2]
    final_poses_clearance[3] = -1 * (theta2 + theta3) 

    print(f"Calculated Clearance Poses (rad): {[round(angle, 3) for angle in final_poses_clearance]}")
    execute_pos(final_poses_clearance, robot_arm, duration_seconds=2.0)
else:
    print("Clearance is out of reach or no solution found.")
    p.disconnect()
    exit()



APPROACH_HEIGHT_OFFSET = 0.1

for container_name, world_pos in rack_positions.items():
    print(f"\nStarting sequence for container: {container_name} ")


    target_pos_tip = [world_pos[0], world_pos[1], world_pos[2] + 0.025] 
    target_pos_ik = [target_pos_tip[0], target_pos_tip[1], target_pos_tip[2] + length_EE]
    approach_pos_tip = [target_pos_tip[0], target_pos_tip[1], target_pos_tip[2] + APPROACH_HEIGHT_OFFSET]
    approach_pos_ik = [approach_pos_tip[0], approach_pos_tip[1], approach_pos_tip[2] + length_EE]

    p.resetBasePositionAndOrientation(target_marker_id, target_pos_tip, p.getQuaternionFromEuler([0, 0, 0]))
    current_joint_state = p.getJointStates(robot_arm, range(4))
    current_joint_angles = [state[0] for state in current_joint_state]
    current_joint_angles_np = np.array(current_joint_angles)
    
    print(f"\nmoving to approach position:{approach_pos_tip}")

    joint_poses_approach_raw = p.calculateInverseKinematics(
        bodyUniqueId=robot_arm,
        endEffectorLinkIndex=EE_LINK_INDEX,
        targetPosition=approach_pos_ik
    )

    final_approach_poses = None
    if joint_poses_approach_raw:
        final_approach_poses = np.array(joint_poses_approach_raw[:4])
        theta2 = final_approach_poses[1]
        theta3 = final_approach_poses[2]
        final_approach_poses[3] = -1 * (theta2 + theta3) 
        
        print(f"Chosen Approach Poses (rad): {[round(angle, 3) for angle in final_approach_poses]}")

        execute_pos(final_approach_poses, robot_arm, duration_seconds=1.5)
    else:
        print("Approach Target out of reach or no solution found. Skipping to next clearance.")
        if final_poses_clearance is not None:
            execute_pos(final_poses_clearance, robot_arm, duration_seconds=2.0)
        continue
    final_target_poses = None
    if final_approach_poses is not None:
        current_joint_angles_np = final_approach_poses
        
        print(f"\nmoving to final target position:{target_pos_tip}")
        

        joint_poses_final_raw = p.calculateInverseKinematics(
            bodyUniqueId=robot_arm,
            endEffectorLinkIndex=EE_LINK_INDEX,
            targetPosition=target_pos_ik
        )
        
        if joint_poses_final_raw:
            final_target_poses = np.array(joint_poses_final_raw[:4])
         
            theta2_final = final_target_poses[1]
            theta3_final = final_target_poses[2]
            theta4_required = -1 * (theta2_final + theta3_final)
            final_target_poses[3] = theta4_required 
            
            print(f"Chosen Final Poses (rad): {[round(angle, 3) for angle in final_target_poses]}")
            print("\nExecuting movement to selected IK target")
            execute_pos(final_target_poses, robot_arm, duration_seconds=1.0) 
            
        else:
           print("Final Target out of reach or no solution found. Skipping to next clearance.")
    print(f"\nReturning to safe clearence pos: {clearance_pos_tip}")
    if final_poses_clearance is not None:
        execute_pos(final_poses_clearance, robot_arm, duration_seconds=2.0)
    
    print(f"Finished sequence for container: {container_name}")


print("\nRunning Simulation Moves")

print("\nMovement Complete")
for i in range(1 * 240): 
    p.stepSimulation()
    time.sleep(1. / 240.) 
    
p.disconnect()
print("\nSimulation Finished. Arm should be at the target.")