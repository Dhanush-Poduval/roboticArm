import math
import matplotlib.pyplot as plt
import numpy as np
import time
from fk import forward_kinematics
L1, L2, L3 = 0.2,0.5,0.5

def inverse_kinematics(x_target,y_target,z_target,angle=0):
    theta1=math.atan2(y_target,x_target)
    r=math.sqrt(x_target**2+y_target**2)
    z_prime=z_target-L1
    rw=math.hypot(r,z_prime)
    if rw>(L2+L3) or rw<abs(L2-L3):
        print(f"Target out of reach ")
        return False
    cos_A3=(L2**2+L3**2-rw**2)/(2*L2*L3)
    cos_A3=max(-1,min(1,cos_A3))
    internal_angle_3=math.acos(cos_A3)
    theta3=-(math.pi-internal_angle_3)

    sin_a2=(L3*math.sin(internal_angle_3))/rw
    sin_a2=max(-1,min(1,sin_a2))
    alpha=math.asin(sin_a2)
    beta=math.atan2(z_prime,r)
    theta2=(beta+alpha)-(math.pi/2)
    theta4=angle
    return (theta1,theta2,theta3,theta4)
'''def plot(angles):
    theta1,theta2,theta3=[math.radians(i) for i in angles]
    x0,y0=0,0
    x1,y1=L1*math.cos(theta1),L1*math.sin(theta1)
    x2,y2=x1+L2*math.cos(theta1+theta2),y1+L2*math.sin(theta1+theta2)
    x3,y3=x2+L3*math.cos(theta1+theta2+theta3),y2+L3*math.sin(theta1+theta2+theta3)
    x4,y4=x3+L4*math.cos(theta1+theta2+theta3),y3+L4*math.sin(theta1+theta2+theta3)
    plt.figure()
    plt.plot([x0,x1,x2,x3,x4],[y0,y1,y2,y3,y4],linewidth=3,markersize=8,marker='o')
    plt.plot(x3,y3,'rx',markersize=10)
    plt.xlim(-250,250)
    plt.ylim(-25,500)
    plt.grid()
    plt.gca().set_aspect('equal',adjustable='box')
    plt.show()
def move_arm(start , end , steps=50):
    
    a1_arr=np.linspace(start[0],end[0],steps)
    a2_arr = np.linspace(start[1], end[1], steps)
    a3_arr= np.linspace(start[2],end[2], steps)
    a4_arr = np.linspace(start[3], end[3], steps)

    for i in range(steps+1):
        a1 = a1_arr[i]
        a2 = a2_arr[i]
        a3 = a3_arr[i]
        a4=a4_arr[i]
        plot((a1, a2, a3,a4))
        time.sleep(0.05)

def automate(targets, steps=50):
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(-300, 300)
    ax.set_ylim(-100, 400)
    ax.set_aspect('equal')
    ax.grid(True)

   
    curr_angles = (0, 0, 0,0)

    for target in targets:
        x, y = target
        angles = inverse_kinematics(x, y)
        if not angles:
            print(f"Target {target} out of reach")
            continue

        print(f"Moving to target {target}...")

       
        theta1_arr = np.linspace(curr_angles[0], angles[0], steps)
        theta2_arr = np.linspace(curr_angles[1], angles[1], steps)
        theta3_arr = np.linspace(curr_angles[2], angles[2], steps)
        theta4_arr = np.linspace(curr_angles[3], angles[3], steps)
        for t1, t2, t3 , t4 in zip(theta1_arr, theta2_arr, theta3_arr,theta4_arr):
            (x0, y0), (x1, y1), (x2, y2), (x3, y3),(x4,y4) = forward_kinematics(t1, t2, t3,t4, L1, L2, L3,L4)
            ax.clear()
            ax.plot([x0, x1, x2, x3 , x4], [y0, y1, y2, y3 , y4], 'o-', linewidth=3, markersize=8)
            ax.plot(x, y, 'rx', markersize=10)
            ax.set_xlim(-300, 300)
            ax.set_ylim(-100, 400)
            ax.set_aspect('equal')
            ax.grid(True)
            plt.draw()
            plt.pause(0.02)

        curr_angles = angles  

    plt.ioff()
    plt.show()
'''

target = [(150, 150), (200, 100), (100, 200)]

angle_list=[]

#automate(target,steps=30)

target_x, target_y, target_z = 0.1,0.3,0.5 # Target from sim.py
angle = 0 # Radians

result= inverse_kinematics(target_x, target_y, target_z, angle)

if result is not False:
    theta1,theta2,theta3,theta4=result
    print(f"\n--- Analytical Solution ---")
    print(f"Target (X, Y, Z): ({target_x}, {target_y}, {target_z})")
    print(f"Theta 1 (Yaw):    {theta1:.3f} rad")
    print(f"Theta 2 (Shoulder): {theta2:.3f} rad")
    print(f"Theta 3 (Elbow):  {theta3:.3f} rad")
    print(f"Theta 4 (Roll):   {theta4:.3f} rad")
else:
    print(f"\n--- Analytical Solution Failed ---")


#This part is for manual checking of fk and ik functions
'''for targets in target:
    angles=inverse_kinematics(*targets)
    angle_list.append(angles)
    print(f'Target:{targets}')
    if angles:
        print(f'Joint Angles :{angles}')
    else:
        print("Target out of reach")
theta1=angle_list[0][0]
theta2=angle_list[0][1]
theta3=angle_list[0][2]
print(theta1,theta2,theta3)
pos=forward_kinematics(theta1,theta2,theta3,L1,L2,L3)
print("The positions of the arm is :",pos)'''
    
        

