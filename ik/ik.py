import math
import matplotlib.pyplot as plt
from fk import forward_kinematics
L1, L2, L3 = 10, 10, 5

def inverse_kinematics(x, y):
    wx = x - L3 * (x / math.sqrt(x**2 + y**2))
    wy = y - L3 * (y / math.sqrt(x**2 + y**2))
    r = math.sqrt(wx**2 + wy**2)
    theta1 = math.atan2(wy, wx)
    cos_angle = (L1**2 + r**2 - L2**2) / (2 * L1 * r)
    cos_angle = max(-1, min(1, cos_angle))
    theta2 = math.acos(cos_angle)
    cos_elbow = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
    cos_elbow = max(-1, min(1, cos_elbow))
    theta3 = math.acos(cos_elbow)
    if x>L1+L2+L3 or y>L1+L2+L3 or r>L1+L2:
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

target =  [(12, 8), (15, 5), (30,0)]
angle_list=[]
for targets in target:
    angles=inverse_kinematics(*targets)
    angle_list.append(angles)
    print(f'Target:{targets}')
    if angles:
        print(f'Joint Angles :{angles}')
        plot(angles)
    else:
        print("Target out of reach")
theta1=angle_list[0][0]
theta2=angle_list[0][1]
theta3=angle_list[0][2]
print(theta1,theta2,theta3)
pos=forward_kinematics(theta1,theta2,theta3,L1,L2,L3)
print("The positions of the arm is :",pos)
    
        

