import numpy as np
from dh import dh_transform
import math

def forward_kinematics(theta1,theta2,theta3,L1,L2,L3):
    rad_1=math.radians(theta1)
    rad_2=math.radians(theta2)
    rad_3=math.radians(theta3)
    A1=dh_transform(L1,0,0,rad_1)
    A2=dh_transform(L2,0,0,rad_2)
    A3=dh_transform(L3,0,0,rad_3)
    T=A1 @ A2 @ A3
    return T[:3,3]
