import numpy as np
import math

def dh_transform(a,alpha,d,theta):
    ca,sa=math.cos(alpha),math.sin(alpha)
    ct,st=math.cos(theta),math.sin(theta)
    return np.array([
        [ct,-st*ca,st*sa,a*ct],
        [st,ct*ca,-ct*sa,a*st],
        [0,sa,ca,d],
        [0,0,0,1]
        ])

