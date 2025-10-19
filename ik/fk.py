import numpy as np
from dh import dh_transform
import math

def forward_kinematics(theta1,theta2,theta3,d1,L2,L3):
    A1=dh_transform(0,math.pi/2,d1,theta1)
    A2=dh_transform(L2,0,0,theta2)
    A3=dh_transform(L3,0,0,theta3)
    T=A1 @ A2 @ A3
    return T[:3,3]
