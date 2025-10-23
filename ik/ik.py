import math
import matplotlib.pyplot as plt
import numpy as np
import time
from fk import forward_kinematics
L1, L2, L3 ,L4= 70,150,180,60

def inverse_kinematics(x, y, phi=0):
    wx = x - L4 * math.cos(phi)
    wy = y - L4 * math.sin(phi)
    rw = math.hypot(wx, wy)
    L23=L2+L3
    if rw > (L1 + L2) or rw < abs(L1 - L2):
        print("Target out of reach")
        return False
   
    cos_theta2 = (rw**2 - L1**2 - L23**2) / (2 * L1 * L23)
    cos_theta2 = max(-1, min(1, cos_theta2))  
    theta2 = math.acos(cos_theta2)  
    theta3 = 0
    theta1 = math.atan2(wy, wx) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    theta4 = phi - (theta1 + theta2+theta3)
    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3),math.degrees(theta4)
def plot(angles):
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


target = [(150, 150), (200, 100), (100, 200)]

angle_list=[]
automate(target,steps=30)

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
    
        

