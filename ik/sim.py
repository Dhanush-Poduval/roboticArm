import pybullet as p 
import os
import time  # For controlling the simulation speed
import pybullet_data
import math  # For mathematical functions like pi

# --- 1. FILE PATH SETUP ---
# Determine the correct path for the URDF file
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
urdf_path = os.path.join(parent_dir, 'four_dof_arm.urdf')

print(f"DEBUG: Final URDF Path: {urdf_path}") 
if not os.path.exists(urdf_path):
    print("FATAL ERROR: The file path is INCORRECT. Cannot proceed.")
    exit()

# --- 2. SETUP PYBULLET SIMULATION ---
physicsClient = p.connect(p.GUI) # Start the GUI
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Load necessary data (e.g., plane)
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf") 

# Load your robot (URDF now has proper inertial tags, fixing previous warnings)
robot_arm = p.loadURDF(urdf_path, useFixedBase=True, basePosition=[0,0,0])

# --- 3. IK PARAMETERS ---
# CRITICAL FIX: The verified index for the 'tool_link' is 3
EE_LINK_INDEX = 3 
num_joints = p.getNumJoints(robot_arm)

print(f"\nRobot Loaded with {num_joints} joints. End-Effector Index: {EE_LINK_INDEX}")

# --- 4. DEFINE TARGET POSE ---
# Target Position (x, y, z) in meters. 
# Target is reachable: 0.7m out on Y axis, 1.0m up on Z axis.
target_pos = [0.0, 0.7, 1.0] 

# Target Orientation (Quaternions). We will not use this for the first successful test.
target_orientation = p.getQuaternionFromEuler([0, -math.pi / 4, 0]) 

# --- 5. CALCULATE AND APPLY IK ---
print("\n--- Running Inverse Kinematics ---")

# Calculate the required joint angles (ignoring orientation for stability)
joint_poses = p.calculateInverseKinematics(
    bodyUniqueId=robot_arm,
    endEffectorLinkIndex=EE_LINK_INDEX,
    targetPosition=target_pos,
    # targetOrientation=target_orientation, # <-- Uncomment this line for the next test
    maxNumIterations=50, # Optional: more iterations for better accuracy
    residualThreshold=1e-4
)

# Apply the calculated joint angles to the motors immediately
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

# --- 6. RUN SIMULATION LOOP ---
print("--- Starting Simulation to move the arm ---")

# Run the simulation for 5 seconds to visualize the movement
sim_time_seconds = 5
for i in range(sim_time_seconds * 240):
    p.stepSimulation()
    time.sleep(1. / 240.) # Pause to stabilize the simulation speed

# --- 7. CLEANUP ---
p.disconnect()
print("\nSimulation Finished. Arm should be at the target.")
