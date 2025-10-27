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
num_joints=p.getNumJoints(robot_arm)
#shelf_half=np.array([0.8,0.4,0.02])
#shelf_pos=np.array([0.0, 1.1, 0.37])
num_joints = p.getNumJoints(robot_arm)
containers={}
JOINT_INDICES = list(range(4))
length_EE = 0.05
APPROACH_HEIGHT_OFFSET = 0.1

def load_obstacle(position,half_extents,color=[0.5,0.5,0.5,1]):
    visual_shape=p.createVisualShape(p.GEOM_BOX,halfExtents=half_extents,rgbaColor=color)
    collision_shape=p.createCollisionShape(p.GEOM_BOX,halfExtents=half_extents)
    obstacle_id=p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
    )
    return obstacle_id
SHELF_THICKNESS = 0.04
SHELF_HEIGHT = 0.8
SHELF_WIDTH = 2.3
SHELF_DEPTH = 0.8
shelf_pos=np.array([0.0,0.1,0.0])
shelf_floor_half=[SHELF_WIDTH/2,SHELF_DEPTH/2,SHELF_THICKNESS/2]
shelf_floor_pos = [shelf_pos[0], shelf_pos[1], shelf_pos[2] + SHELF_THICKNESS / 2]
shelf_floor_id = load_obstacle(shelf_floor_pos, shelf_floor_half, color=[0.5, 0.5, 0.5, 0.7])
shelf_side_half = [SHELF_THICKNESS / 2, SHELF_DEPTH / 2, SHELF_HEIGHT / 2]
side_center_z = shelf_floor_pos[2] + shelf_floor_half[2] + SHELF_HEIGHT / 2
left_side_x = shelf_pos[0] - SHELF_WIDTH / 2 + SHELF_THICKNESS / 2
left_wall_pos = [left_side_x, shelf_pos[1], side_center_z]
shelf_left_id = load_obstacle(left_wall_pos, shelf_side_half, color=[0.8, 0.2, 0.2, 0.7])
right_side_x = shelf_pos[0] + SHELF_WIDTH / 2 - SHELF_THICKNESS / 2
right_wall_pos = [right_side_x, shelf_pos[1], side_center_z]
shelf_right_id = load_obstacle(right_wall_pos, shelf_side_half, color=[0.2, 0.2, 0.8, 0.7])
SHELF_OBSTACLE_IDS = [shelf_floor_id, shelf_left_id, shelf_right_id]
container_base_z=shelf_floor_pos[2]+shelf_floor_half[2]

rack_positions={
    'C07': (-0.4, 0.9, container_base_z+0.5), 
    'C08': (0.0, 0.9, container_base_z+0.5),
    'C09': (0.4, 0.9, container_base_z+0.5),
}

x_min_safe=-0.70
x_max_safe=0.70
y_app_min=0.3
y_app_max=1.8
z_min_safe=0.1
def valid_ee(joint_angles,robot_arm,ee_link):
    for j in JOINT_INDICES:
        p.resetJointState(robot_arm, j, joint_angles[j])

    link_state = p.getLinkState(robot_arm,EE_LINK_INDEX)
    ee_pos = link_state[0]
    x, y, z = ee_pos[0], ee_pos[1], ee_pos[2]
    if x < x_min_safe or x > x_max_safe:
        print(f"\tConstraint Violation: X={x:.3f} outside [{x_min_safe}, {x_max_safe}] (Shelf Side)")
        return False
    if y < y_app_min or y > y_app_max:
        print(f"\tConstraint Violation: Y={y:.3f} outside [{y_app_min}, {y_app_max}] (Shelf Depth)")
        return False
        
    if z < z_min_safe:
        print(f"\tConstraint Violation: Z={z:.3f} below {z_min_safe}")
        return False
        
    return True
def check_collision(id,obstacle_id,dist=0.01):
    for obstacle_id in obstacle_id:
        for link_index in range(-1,p.getNumJoints(id)):
            closest_points=p.getClosestPoints(
                bodyA=id,
                bodyB=obstacle_id,
                distance=dist,
                linkIndexA=link_index,
                linkIndexB=-1
            )
            if closest_points:
                for point in closest_points:
                    if point[8]<=dist:
                        return True
    return False

def execute_safe_pos(joint_target,robot_arm,duration=1.0,sim_time_per_step=1.0/240.0):
    current_joint_state=p.getJointStates(robot_arm,JOINT_INDICES)
    current_joint_angles=np.array([state[0]for state in current_joint_state])
    target_joint_angles=np.array(joint_target)
    steps=int(duration/sim_time_per_step)
    if steps<2:steps=2
    for i in range(steps):
        alpha=i/(steps-1)
        interpolated_pos=current_joint_angles*(1-alpha)+target_joint_angles*alpha
        interpolated_pos[3] = -1 * (interpolated_pos[1] + interpolated_pos[2])
        for joint_index in JOINT_INDICES:
            p.resetJointState(
                bodyUniqueId=robot_arm,
                jointIndex=joint_index,
                targetValue=interpolated_pos[joint_index]
            )
        if check_collision(robot_arm,SHELF_OBSTACLE_IDS):
            print(f"Path has some blockage")
            return False
        for joint_index in JOINT_INDICES:
         p.setJointMotorControl2(
            bodyUniqueId=robot_arm,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=interpolated_pos[joint_index]
         )
        p.stepSimulation()
        time.sleep(sim_time_per_step)
    return True
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
    return execute_safe_pos(joint_target,robot_arm_id,duration_seconds,sim_time_per_step)
    '''
    Old logic without collision detection
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
    '''
for name,pos in rack_positions.items():
   containers[name]=load_container(name,pos)

print(f"Robot Loaded with {num_joints} joints. End-Effector Index: {EE_LINK_INDEX}")

target_pos_tip = [-0.4,0.85,0.39] 

length_EE = 0.05


target_pos_ik = [target_pos_tip[0], target_pos_tip[1], target_pos_tip[2]]

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
    if not valid_ee(final_poses_clearance,robot_arm,EE_LINK_INDEX):
        print("Initial clearence violates constraints")
        p.disconnect()
        exit()
    
    print(f"Calculated Clearance Poses (rad): {[round(angle, 3) for angle in final_poses_clearance]}")
    if not  execute_pos(final_poses_clearance, robot_arm, duration_seconds=2.0):
        print("Fail due to path collision")
        p.disconnect()
        exit()
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
        if not valid_ee(final_approach_poses,robot_arm,EE_LINK_INDEX):
            print("Contraints have been violated in this path")
            if final_poses_clearance is not None: execute_pos(final_poses_clearance, robot_arm, duration_seconds=1.0)
            continue
        print(f"Chosen Approach Poses (rad): {[round(angle, 3) for angle in final_approach_poses]}")

        if not execute_pos(final_approach_poses, robot_arm, duration_seconds=1.5):
            print("Approach Move FAILED due to path collision. Returning to clearance.")
            if final_poses_clearance is not None: execute_pos(final_poses_clearance, robot_arm, duration_seconds=1.0)
            continue
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
            if not valid_ee(final_target_poses, robot_arm, EE_LINK_INDEX):
                print("Final Target Pose violates spatial constraints. Skipping.")
            
            else: 
                print(f"Chosen Final Poses (rad): {[round(angle, 3) for angle in final_target_poses]}")
                print("\nExecuting movement to selected IK target")
                if not execute_pos(final_target_poses, robot_arm, duration_seconds=1.0):
                    print("Final Target Move FAILED due to path collision.")
            print(f"Chosen Final Poses (rad): {[round(angle, 3) for angle in final_target_poses]}")
            print("\nExecuting movement to selected IK target")
            execute_pos(final_target_poses, robot_arm, duration_seconds=1.0) 
            
        else:
           print("Final Target out of reach .")
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