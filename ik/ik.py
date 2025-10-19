import math


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
    return math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)
target = (12, 8)
angles = inverse_kinematics(*target)
print("Target:", target)
print("Joint Angles:", angles)

