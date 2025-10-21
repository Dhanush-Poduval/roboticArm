import math
import matplotlib.pyplot as plt
import numpy as np
import time
from fk import forward_kinematics
L1, L2, L3 = 10, 10, 5

def inverse_kinematics(x, y):
    r = math.sqrt(x**2 + y**2)
    wx=x-L3*(x/r)
    wy=y-L3*(y/r)   
    rw=math.sqrt(wx**2 + wy**2)
    cos_theta2 = (rw**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(-1, min(1, cos_theta2))
    theta2 = math.acos(cos_theta2)
    theta1 = math.atan2(wy, wx)-math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    theta3 = math.atan2(y - wy, x - wx) - (theta1 + theta2)
    if r>L1+L2+L3:
        return False
    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)
def plot(angles):
    theta1,theta2,theta3=[math.radians(i) for i in angles]
    x0,y0=0,0
    x1,y1=L1*math.cos(theta1),L1*math.sin(theta1)
    x2,y2=x1+L2*math.cos(theta1+theta2),y1+L2*math.sin(theta1+theta2)
    x3,y3=x2+L3*math.cos(theta1+theta2+theta3),y2+L3*math.sin(theta1+theta2+theta3)
    plt.figure()
    plt.plot([x0,x1,x2,x3],[y0,y1,y2,y3],linewidth=3,markersize=8,marker='o')
    plt.plot(x3,y3,'rx',markersize=10)
    plt.xlim(-30,30)
    plt.ylim(-5,30)
    plt.grid()
    plt.gca().set_aspect('equal',adjustable='box')
    plt.show()
def move_arm(start , end , steps=50):
    
    a1_arr=np.linspace(start[0],end[0],steps)
    a2_arr = np.linspace(start[1], end[1], steps)
    a3_arr= np.linspace(start[2],end[2], steps)
    for i in range(steps+1):
        a1 = a1_arr[i]
        a2 = a2_arr[i]
        a3 = a3_arr[i]
        plot((a1, a2, a3))
        time.sleep(0.05)
def automate(targets,steps=50):
    plt.ion()
    fig,ax=plt.subplots()
    ax.set_xlim(-25,25)
    ax.set_ylim(-5,25)
    ax.set_aspect('equal')
    ax.grid(True)

    for target in targets:
        x,y=target
        angles=inverse_kinematics(x,y)
        if not angles:
            print(f"Taget {target} out of reach")
        theta1,theta2,theta3=angles
        print(f"Moving to target {target} lesgo")
        for i in np.linspace(0,1 ,steps):
            (x0,y0),(x1,y1),(x2,y2)=forward_kinematics(theta1*i,theta2*i,theta3*i,L1,L2,L3)
            ax.clear()
            ax.plot([x0,x1,x2],[y0,y1,y2],'o-',linewidth=3,markersize=8)
            ax.plot(x,y,'rx',markersize=10)
            ax.set_ylim(-5, 25)
            ax.set_aspect('equal')
            ax.grid(True)
            plt.draw()
            plt.pause(0.02)
    plt.ioff()
    plt.show()

        


target =  [(13,9), (15, 5), (1,6)]
angle_list=[]
automate(target)
'''for targets in target:
    angles=inverse_kinematics(*targets)
    angle_list.append(angles)
    print(f'Target:{targets}')
    if angles:
        print(f'Joint Angles :{angles}')
    else:
        print("Target out of reach")
for i in range(len(angle_list)):
    start = angle_list[i]
    end = angle_list[i + 1]
    if start and end:
        move_arm(start, end)
theta1=angle_list[0][0]
theta2=angle_list[0][1]
theta3=angle_list[0][2]
print(theta1,theta2,theta3)
pos=forward_kinematics(theta1,theta2,theta3,L1,L2,L3)
print("The positions of the arm is :",pos)'''
    
        

