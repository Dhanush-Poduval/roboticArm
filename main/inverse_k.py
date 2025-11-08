import math
import matplotlib.pyplot as plt
import numpy as np
import time
# Assuming fk.py and forward_kinematics are available
# from fk import forward_kinematics 

# --- UPDATED CONSTANTS ---
L1 = 0.1  # Base height/offset (assuming this is part of the Z clearance)
L2 = 0.5  # Link 2 length (Shoulder to Elbow)
L3 = 0.5  # Link 3 length (Elbow to Wrist)
L_EE = 0.10 # End-Effector (Gripper) Length - Must match PyBullet code

J2_LOWER = -1.57 
J2_UPPER = 1.57
J3_LOWER = -2.0
J3_UPPER = 2.0
# -------------------------

def is_valid(theta2,theta3):
    if not (J2_LOWER<=theta2<=J2_UPPER):
        return False
    if not (J3_LOWER <= theta3 <= J3_UPPER):
        return False
    return True

def inverse_kinematics(x_target, y_target, z_target):
    """
    Calculates the joint angles (theta1, theta2, theta3, theta4) for the 
    4-DoF arm, ensuring the wrist is vertical (theta4 = -(theta2 + theta3)).
    
    The target (x, y, z) is the desired position of the gripper tip.
    The IK solution is calculated for the J4 position, offset by L_EE.
    """
    
    # 1. Target Offset: Adjust Z-target to find the position of Joint 4 (J4)
    # P_J4 must be L_EE below the target P_EE to keep the gripper vertical.
    z_prime_J4 = z_target - L_EE # New Z for the 3-link planar solution
    
    # 2. Theta 1 (Base Rotation)
    theta1 = math.atan2(y_target, x_target)
    
    # 3. Planar Distance and Hypotenuse
    r = math.hypot(x_target, y_target)
    rw = math.hypot(r, z_prime_J4)
    
    # 4. Reachability Check
    if rw > (L2 + L3) or rw < abs(L2 - L3):
        print(f"Target out of reach (Distance: {rw:.3f}m)")
        return False
    
    # 5. Theta 3 (Elbow Angle - Law of Cosines)
    cos_A3 = (L2**2 + L3**2 - rw**2) / (2 * L2 * L3)
    cos_A3 = max(-1, min(1, cos_A3))
    internal_angle_3 = math.acos(cos_A3) # Internal triangle angle

    # The two solutions for Theta 3: Elbow Up and Elbow Down
    # Elbow Down (typically positive angle for forward reach, assuming PyBullet URDF)
    theta3_down = math.pi - internal_angle_3 
    # Elbow Up (typically negative angle)
    theta3_up = -(math.pi - internal_angle_3) 

    # 6. Theta 2 components (Angle of elevation and correction angle)
    beta = math.atan2(z_prime_J4, r)
    sin_alpha = (L3 * math.sin(internal_angle_3)) / rw
    sin_alpha = max(-1, min(1, sin_alpha))
    alpha = math.asin(sin_alpha)

    # The two solutions for Theta 2:
    theta2_down = beta + alpha  # Corresponds to Elbow Down
    theta2_up = beta - alpha    # Corresponds to Elbow Up

    # --- Solution Types ---
    
    # Solution 1: Elbow Up
    theta4_up = -1 * (theta2_up + theta3_up) # Calculated from the constraint
    up_type=[theta1, theta2_up, theta3_up, theta4_up]

    # Solution 2: Elbow Down
    theta4_down = -1 * (theta2_down + theta3_down) # Calculated from the constraint
    down_type=[theta1, theta2_down, theta3_down, theta4_down]

    valid_solutions=[]

    if is_valid(theta2_up, theta3_up):
        valid_solutions.append(up_type)
    if is_valid(theta2_down, theta3_down):
        valid_solutions.append(down_type)
        
    if not valid_solutions:
        print("Target is making joints move outside the limit ")
        return False
    
    # For a simple choice, return the first valid solution found (or down-type if both valid)
    return valid_solutions[0] if valid_solutions else False 

# --- End of Updated IK Function ---

# The plotting and manual checking code remains the same as you provided, 
# but it should now call the single-solution IK function:

'''
# Example of how to use the updated function:
target_x, target_y, target_z = 0.1,0.3,0.5 
result = inverse_kinematics(target_x, target_y, target_z)

if result is not False:
    theta1,theta2,theta3,theta4=result
    print(f"\n--- Analytical Solution ---")
    print(f"Target (X, Y, Z): ({target_x}, {target_y}, {target_z})")
    print(f"Theta 1 (Yaw):    {theta1:.3f} rad")
    print(f"Theta 2 (Shoulder): {theta2:.3f} rad")
    print(f"Theta 3 (Elbow):  {theta3:.3f} rad")
    print(f"Theta 4 (Wrist):  {theta4:.3f} rad (Constraint: -(T2+T3))")
else:
    print(f"\n--- Analytical Solution Failed ---")
'''

# The rest of your script follows here...