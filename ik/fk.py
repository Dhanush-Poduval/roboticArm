import numpy as np
import math

def forward_kinematics(theta1, theta2, theta3 ,theta4,L1 ,L2 ,L3,L4):
    
    t1 = math.radians(theta1)
    t2 = math.radians(theta2)
    t3 = math.radians(theta3)
    t4=math.radians(theta4)
    x0=0
    y0=0
    x1 = L1 * math.cos(t1)
    y1 = L1 * math.sin(t1)

    x2 = x1 + L2 * math.cos(t1 + t2)
    y2 = y1 + L2 * math.sin(t1 + t2)

    x3 = x2 + L3 * math.cos(t1 + t2 + t3)
    y3 = y2 + L3 * math.sin(t1 + t2 + t3)

    x4=x3+L4*math.cos(t1+t2+t3+t4)
    y4=y3+L4*math.sin(t1+t2+t3+t4)
    
    return (x0,y0),(x1, y1), (x2, y2), (x3, y3),(x4,y4)#this is just to check the position of like all the joints of the arm 
